import re
import time
import logging
import rospy

logger = logging.getLogger(__name__)


def detect_topics_from_sdf(sdf_path: str) -> list[str]:
    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error("detect_topics_from_sdf: cannot read %r: %s", sdf_path, e)
        return []

    seen = set()
    topics = []

    namespace = ""
    namespace_match = re.search(r"<namespace>\s*(/[^<\s]+)\s*</namespace>", content)
    if namespace_match:
        namespace = namespace_match.group(1).rstrip('/')

    for m in re.finditer(r"<\w*[Tt]opic\w*>\s*(/[^<\s]+)\s*</\w*[Tt]opic\w*>", content):
        t = m.group(1).strip()
        if t and t not in seen:
            seen.add(t)
            topics.append(t)

    for m in re.finditer(r"<argument>\s*([^<]+?)\s*</argument>", content):
        arg = m.group(1).strip()

        if ":= " in arg or ":=" in arg:
            parts = re.split(r":=\s*", arg)
            if len(parts) == 2:
                remapped = parts[1].strip()
                if not remapped.startswith('/') and namespace:
                    topic = f"{namespace}/{remapped}"
                elif remapped.startswith('/'):
                    topic = remapped
                else:
                    topic = remapped

                if topic and topic not in seen:
                    seen.add(topic)
                    topics.append(topic)
        else:
            if not arg.startswith('/') and namespace:
                topic = f"{namespace}/{arg}"
            elif arg.startswith('/'):
                topic = arg
            else:
                topic = arg

            if topic and topic not in seen and not topic.startswith('--') and not topic.startswith('-'):
                seen.add(topic)
                topics.append(topic)

    for m in re.finditer(r"<(\w*[Tt]opic\w*)>\s*([^<\s]+)\s*</\1>", content):
        topic_name = m.group(2).strip()
        if not topic_name.startswith('/') and namespace:
            topic = f"{namespace}/{topic_name}"
        elif topic_name.startswith('/'):
            topic = topic_name
        else:
            topic = topic_name

        if topic and topic not in seen and not topic.startswith('--'):
            seen.add(topic)
            topics.append(topic)

    return topics


class Sensor:
    def __init__(self, sensor_type: str, sensor_name: str, sdf_path: str,
                 topics: list = None, description: str = "",
                 image_path: str = "", params: dict = None):
        self.sensor_type = sensor_type
        self.sensor_name = sensor_name
        self.sdf_path    = sdf_path
        self.topics      = list(topics or [])
        self.description = description
        self.image_path  = image_path
        self.params      = params or {}


    @property
    def topic(self) -> str:
        """Primary topic — first in list, for backwards compat."""
        return self.topics[0] if self.topics else ""


    def param(self, name: str, default=None):
        return self.params.get(name, default)


    def read_params_from_sdf(self, param_defs) -> dict:
        """Read parameter values from the SDF file.

        param_defs can be:
          - list[str]  — legacy flat tag names, e.g. ["width", "height"]
          - list[dict] — {"name": "width"} or {"name": "width", "path": "camera"}
                         path supports arbitrary depth: "camera/lens"

        Returns {param_name: value}.
        """
        if not self.sdf_path:
            return {}
        try:
            with open(self.sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError as e:
            logger.error("read_params_from_sdf: cannot read %r: %s", self.sdf_path, e)
            return {}

        result = {}
        for item in param_defs:
            if isinstance(item, str):
                name = item
                path = ""
            else:
                name = item.get("name", "")
                path = item.get("path", "")

            if not name:
                continue

            search_in = content
            if path:
                # Walk down each segment of the path, scoping progressively deeper
                for seg in path.split("/"):
                    seg_m = re.search(
                        rf"<{re.escape(seg)}>(.*?)</{re.escape(seg)}>",
                        search_in, re.DOTALL
                    )
                    if seg_m:
                        search_in = seg_m.group(1)
                    else:
                        search_in = None
                        break
                if search_in is None:
                    continue

            m = re.search(
                rf"<{re.escape(name)}>\s*(.*?)\s*</{re.escape(name)}>",
                search_in, re.DOTALL
            )
            if m:
                result[name] = m.group(1).strip()

        return result


    def write_params_to_sdf(self, params: dict) -> None:
        if not self.sdf_path:
            raise ValueError("sdf_path is not set")
        try:
            with open(self.sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError as e:
            raise OSError(f"Cannot read SDF: {e}") from e
        for name, value in params.items():
            content, n = re.subn(
                rf"(<{re.escape(name)}>)\s*.*?\s*(</{re.escape(name)}>)",
                rf"\g<1>{value}\g<2>",
                content, count=1, flags=re.DOTALL,
            )
            if n == 0:
                logger.warning("write_params_to_sdf: tag <%s> not found in %s", name, self.sdf_path)
        with open(self.sdf_path, "w", encoding="utf-8") as f:
            f.write(content)
        self.params.update(params)


    def capture_data(self, msg_type, topic: str = "", window: float = 2.0,
                     timeout: float = 0.25, warmup: float = 0.0,
                     simulator=None) -> list:
        """
        Read messages from a ROS topic for `window` seconds.
        Uses `topic` if given, otherwise falls back to self.topic (first in list).
        Returns a list of raw ROS messages in arrival order.

        warmup: seconds to sleep before starting the capture loop.
                Use this for sensors (e.g. cameras) whose plugin takes a few
                seconds to register its ROS topic after Gazebo starts.

        After capture: fires simulator.notify_capture and respects step-mode.
        """
        t        = topic or self.topic
        results  = []
        if warmup > 0:
            time.sleep(warmup)
        deadline = time.time() + window
        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(t, msg_type, timeout=timeout)
                results.append(msg)
            except Exception:
                continue

        if simulator is not None:
            sensor_data = {
                "sensor_type": self.sensor_type,
                "sensor_name": self.sensor_name,
                "topic":       t,
                "count":       len(results),
                "image_path":  self.image_path,  # catalog photo from DB
                "messages":    results,   # raw ROS messages for processing
            }
            obs_img = simulator.capture_observer_frame() if simulator.gazebo_is_running else None
            simulator.notify_capture(sensor_data, obs_img)
            simulator.wait_for_step()

        return results


    def __repr__(self):
        return f"<Sensor {self.sensor_type!r} name={self.sensor_name!r}>"
