import re
import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


def detect_topics_from_sdf(sdf_path: str) -> list[str]:
    """Extract ROS topics from an SDF file.

    Handles multiple plugins per SDF (e.g. stereo cameras with two
    libgazebo_ros_camera.so instances).  Each <plugin> block is processed
    independently — its own namespace/cameraName/robotNamespace is used
    to form fully-qualified topic paths.

    Topics containing 'image_raw' are sorted to the front of the list
    because camera tests rely on them as the primary data source.
    """
    try:
        with open(sdf_path, "r", encoding="utf-8") as f:
            content = f.read()
    except OSError as e:
        logger.error("detect_topics_from_sdf: cannot read %r: %s", sdf_path, e)
        return []

    seen = set()
    topics = []

    def _add(topic: str) -> None:
        if topic and topic not in seen and not topic.startswith("-"):
            seen.add(topic)
            topics.append(topic)

    def _qualify(name: str, ns: str) -> str:
        """Prepend namespace if the name is not already absolute."""
        if name.startswith("/"):
            return name
        if ns:
            return f"{ns.rstrip('/')}/{name.lstrip('/')}"
        return f"/{name}"

    # ── Process each <plugin> block independently ────────────────────
    for plugin_m in re.finditer(
        r"<plugin\b[^>]*>(.*?)</plugin>", content, re.DOTALL
    ):
        block = plugin_m.group(1)

        # Determine the namespace for this plugin block.
        # Priority: <robotNamespace> > <namespace> > <cameraName>
        ns = ""
        rns = re.search(r"<robotNamespace>\s*(.*?)\s*</robotNamespace>", block)
        if rns:
            ns = rns.group(1).strip().rstrip("/")
        else:
            ns_m = re.search(r"<namespace>\s*(.*?)\s*</namespace>", block)
            if ns_m:
                ns = ns_m.group(1).strip().rstrip("/")

        # <cameraName> — Gazebo camera plugin uses this as topic prefix:
        #   /cameraName/image_raw, /cameraName/camera_info
        cam_name = re.search(r"<cameraName>\s*(.*?)\s*</cameraName>", block)
        if cam_name:
            cam = cam_name.group(1).strip()
            if cam:
                # cameraName overrides namespace for libgazebo_ros_camera topics
                _add(f"/{cam}/image_raw")
                _add(f"/{cam}/camera_info")

        # <topicName> / <*Topic*> — explicit topic tags
        for tm in re.finditer(r"<(\w*[Tt]opic\w*Name?)>\s*([^<\s]+)\s*</\1>", block):
            _add(_qualify(tm.group(2).strip(), ns))

        # <argument>xxx:=yyy</argument> — ROS remapping
        for am in re.finditer(r"<argument>\s*([^<]+?)\s*</argument>", block):
            arg = am.group(1).strip()
            if ":=" in arg:
                parts = re.split(r":=\s*", arg)
                if len(parts) == 2:
                    remapped = parts[1].strip()
                    # If cameraName exists, topics are under /cameraName/
                    if cam_name and cam_name.group(1).strip():
                        _add(f"/{cam_name.group(1).strip()}/{remapped}")
                    else:
                        _add(_qualify(remapped, ns))

    # ── Fallback: scan entire file for topic-like tags outside <plugin> ──
    # (catches standalone <topic>/path</topic> etc.)
    for m in re.finditer(r"<\w*[Tt]opic\w*>\s*(/[^<\s]+)\s*</\w*[Tt]opic\w*>", content):
        _add(m.group(1).strip())

    # ── Sort: image_raw topics first (primary data source for tests) ──
    def _sort_key(t: str) -> tuple:
        if "image_raw" in t:
            return (0, t)
        return (1, t)

    topics.sort(key=_sort_key)
    return topics


class Sensor:
    def __init__(
        self,
        sensor_type: str,
        sensor_name: str,
        sdf_path: str,
        topics: list = None,
        description: str = "",
        image_path: str = "",
        params: dict = None,
    ):
        self.sensor_type = sensor_type
        self.sensor_name = sensor_name
        self.sdf_path = sdf_path
        self.topics = list(topics or [])
        self.description = description
        self.image_path = image_path
        self.params = params or {}

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

        Returns {param_name: value}. Values are automatically converted to:
        - float if the string is a single number
        - list of floats if the string contains space-separated numbers
        - string otherwise
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
                raw_name = item
                path = ""
            else:
                raw_name = item.get("name", "")
                path = item.get("path", "")

            if not raw_name:
                continue

            # Support path in the name itself: "collision/geometry/box/size"
            # The last segment is the tag name, everything before is the path.
            if "/" in raw_name and not path:
                parts = raw_name.strip("/").split("/")
                name = parts[-1]
                path = "/".join(parts[:-1])
            else:
                name = raw_name

            if not name:
                continue

            search_in = content
            if path:
                # Walk down each segment of the path, scoping progressively deeper.
                # Use findall to collect ALL matching regions (for multi-plugin SDFs
                # where e.g. two <collision> blocks exist).
                regions = [content]
                for seg in path.split("/"):
                    next_regions = []
                    for region in regions:
                        for m in re.finditer(
                            rf"<{re.escape(seg)}\b[^>]*>(.*?)</{re.escape(seg)}>",
                            region,
                            re.DOTALL,
                        ):
                            next_regions.append(m.group(1))
                    regions = next_regions
                    if not regions:
                        break
                if not regions:
                    continue
                search_in = "\n".join(regions)

            matches = re.findall(
                rf"<{re.escape(name)}>\s*(.*?)\s*</{re.escape(name)}>",
                search_in,
                re.DOTALL,
            )
            if not matches:
                continue

            for idx, raw_val in enumerate(matches):
                value_str = raw_val.strip()
                # If multiple occurrences (e.g. two plugins with same tag),
                # prefix with "1: ", "2: " to distinguish them.
                key = name if len(matches) == 1 else f"{idx + 1}: {name}"

                if value_str:
                    parts = value_str.split()
                    if len(parts) == 1:
                        try:
                            result[key] = float(parts[0])
                        except ValueError:
                            result[key] = value_str
                    else:
                        try:
                            result[key] = [float(p) for p in parts]
                        except ValueError:
                            result[key] = value_str
                else:
                    result[key] = value_str

        return result

    def write_params_to_sdf(self, params: dict) -> None:
        if not self.sdf_path:
            raise ValueError("sdf_path is not set")
        try:
            with open(self.sdf_path, "r", encoding="utf-8") as f:
                content = f.read()
        except OSError as e:
            raise OSError(f"Cannot read SDF: {e}") from e

        for raw_key, value in params.items():
            # Parse "N: tag_name" format for multi-plugin params;
            # fall back to plain tag name (occurrence=1).
            idx_match = re.match(r"^(\d+):\s*(.+)$", raw_key)
            if idx_match:
                occurrence = int(idx_match.group(1))  # 1-based
                name = idx_match.group(2)
            else:
                occurrence = 1
                name = raw_key

            if isinstance(value, list):
                value_str = " ".join(str(v) for v in value)
            elif isinstance(value, (int, float)):
                value_str = str(value)
            else:
                value_str = str(value)

            if occurrence == 1:
                # Fast path: replace first occurrence (same as before)
                content, n = re.subn(
                    rf"(<{re.escape(name)}>)\s*.*?\s*(</{re.escape(name)}>)",
                    rf"\g<1>{value_str}\g<2>",
                    content,
                    count=1,
                    flags=re.DOTALL,
                )
            else:
                # Replace the Nth occurrence
                pattern = rf"(<{re.escape(name)}>)\s*.*?\s*(</{re.escape(name)}>)"
                all_matches = list(re.finditer(pattern, content, re.DOTALL))
                if occurrence <= len(all_matches):
                    m = all_matches[occurrence - 1]
                    content = (
                        content[: m.start()]
                        + f"<{name}>{value_str}</{name}>"
                        + content[m.end() :]
                    )
                    n = 1
                else:
                    n = 0

            if n == 0:
                logger.warning(
                    "write_params_to_sdf: tag <%s> (occurrence %d) not found in %s",
                    name, occurrence, self.sdf_path,
                )

        with open(self.sdf_path, "w", encoding="utf-8") as f:
            f.write(content)
        self.params.update(params)

    def capture_data(
        self,
        msg_type,
        topic: str = "",
        window: float = 2.0,
        timeout: float = 0.25,
        warmup: float = 0.0,
        simulator=None,
    ) -> list:
        """
        Read messages from a ROS topic for `window` seconds.
        Uses `topic` if given, otherwise falls back to self.topic (first in list).
        Returns a list of raw ROS messages in arrival order.

        warmup: seconds to sleep before starting the capture loop.
                Use this for sensors (e.g. cameras) whose plugin takes a few
                seconds to register its ROS topic after Gazebo starts.

        After capture: fires simulator.notify_capture and respects step-mode.
        """
        import rospy

        t = topic or self.topic
        results = []
        if warmup > 0:
            time.sleep(warmup)
        deadline = time.time() + window
        while time.time() < deadline:
            try:
                msg = rospy.wait_for_message(t, msg_type, timeout=timeout)
                # For large image messages, keep only the latest to prevent OOM.
                # A 4K RGB frame is ~28MB; accumulating 30fps for 3s = ~2.5GB.
                msg_bytes = len(getattr(msg, "data", b""))
                if msg_bytes > 2_000_000:  # >2MB per message — likely high-res image
                    results = [msg]  # replace, don't accumulate
                else:
                    results.append(msg)
            except Exception:
                continue

        if simulator is not None:
            sensor_data = {
                "sensor_type": self.sensor_type,
                "sensor_name": self.sensor_name,
                "topic": t,
                "count": len(results),
                "image_path": self.image_path,
                "messages": results[-1:],  # only last message for UI rendering
            }
            obs_img = (
                simulator.capture_observer_frame()
                if simulator.gazebo_is_running
                else None
            )
            simulator.notify_capture(sensor_data, obs_img)
            simulator.wait_for_step()

        return results

    def capture_persistent(
        self,
        msg_type,
        topic: str = "",
        window: float = 2.0,
        simulator=None,
    ) -> list:
        """
        Capture messages using a persistent subscriber (no subscribe/unsubscribe gaps).

        Unlike capture_data (which uses rospy.wait_for_message in a loop and
        loses burst-published messages), this method keeps a single subscriber
        open for the entire window.  Essential for RFID antennas that publish
        one message per detected tag in quick succession — wait_for_message
        catches only the first, missing the rest.

        Returns a list of raw ROS messages in arrival order.
        """
        import rospy

        t = topic or self.topic
        results = []

        def _cb(msg):
            results.append(msg)

        sub = rospy.Subscriber(t, msg_type, _cb, queue_size=500)
        try:
            time.sleep(window)
        finally:
            sub.unregister()

        if simulator is not None:
            sensor_data = {
                "sensor_type": self.sensor_type,
                "sensor_name": self.sensor_name,
                "topic": t,
                "count": len(results),
                "image_path": self.image_path,
                "messages": results[-1:],
            }
            obs_img = (
                simulator.capture_observer_frame()
                if simulator.gazebo_is_running
                else None
            )
            simulator.notify_capture(sensor_data, obs_img)
            simulator.wait_for_step()

        return results

    def capture_frames(
        self,
        msg_type,
        topic: str = "",
        window: float = 2.0,
        timeout: float = 0.25,
        simulator=None,
    ) -> dict:
        """
        RFID-style capture: returns {frame_id: pose} deduplicating by frame_id.
        Uses persistent subscriber to catch all tags published in bursts.
        """
        msgs = self.capture_persistent(
            msg_type, topic=topic, window=window, simulator=simulator
        )
        return {
            msg.header.frame_id: msg.pose
            for msg in msgs
            if hasattr(msg, "header") and hasattr(msg, "pose")
        }

    def __repr__(self):
        return f"<Sensor {self.sensor_type!r} name={self.sensor_name!r}>"
