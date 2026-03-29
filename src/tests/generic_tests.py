"""Generic and debug sensor tests."""

import logging
import os
import time
from typing import Any, Dict, List

import cv2
import numpy as np
import rospy
import rostopic
from sensor_msgs.msg import Image

from ._common import (
    Worlds,
    _camera_classify_sensor_profile,
    _camera_load_sensor_profile,
    _camera_worlds_root,
    _img_to_numpy,
    _world_from_db,
)

logger = logging.getLogger(__name__)


def sensor_capture_basic(simulator, sensor, progress_cb=None) -> dict:
    logger.info("=" * 60)
    logger.info(
        "TESTING SENSOR: %s (%s)",
        getattr(sensor, "sensor_name", "unknown"),
        getattr(sensor, "sensor_type", "unknown"),
    )
    logger.info("TOPIC: %s", sensor.topic)
    logger.info("=" * 60)

    result = {
        "passed": False,
        "topic": sensor.topic,
        "sensor_type": getattr(sensor, "sensor_type", "unknown"),
        "sensor_name": getattr(sensor, "sensor_name", "unknown"),
        "duration": 0,
        "message_type": None,
        "data_received": False,
        "error": None,
    }

    t0 = time.time()

    world_path = Worlds.BASIC
    logger.debug("World path: %s", world_path)
    logger.debug("World exists: %s", os.path.exists(world_path))
    logger.debug("Sensor SDF path: %s", sensor.sdf_path)
    logger.debug("Sensor SDF exists: %s", os.path.exists(sensor.sdf_path))

    if not os.path.exists(world_path):
        result["error"] = f"World file not found: {world_path}"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("World file not found: %s", world_path)
        return result

    logger.info("Opening Gazebo scene...")
    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Failed to open Gazebo scene")
        return result

    logger.info("Gazebo scene opened successfully")

    if progress_cb:
        try:
            progress_cb(20)
            logger.debug("Progress callback 20%%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    logger.debug("Waiting 3 seconds for plugins to initialize...")
    time.sleep(3)

    logger.debug("Getting published topics...")
    topics = rospy.get_published_topics()
    logger.debug("Found %d topics total", len(topics))

    logger.debug("Available topics:")
    for t, t_type in topics[:15]:
        if not t.startswith("/rosout"):
            logger.debug("        %s -> %s", t, t_type)

    topic_exists = any(t == sensor.topic for t, _ in topics)
    logger.debug("Topic '%s' exists: %s", sensor.topic, topic_exists)

    if not topic_exists:
        result["error"] = f"Topic {sensor.topic} not found"
        result["available_topics"] = [
            t for t, _ in topics[:20] if not t.startswith("/rosout")
        ]
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Topic not found!")
        return result

    if progress_cb:
        try:
            progress_cb(40)
            logger.debug("Progress callback 40%%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    logger.debug("Getting message class for topic: %s", sensor.topic)
    msg_class, real_topic, _ = rostopic.get_topic_class(sensor.topic)
    logger.debug("rostopic.get_topic_class returned: %s", msg_class)
    logger.debug("Real topic: %s", real_topic)

    if msg_class is None:
        logger.debug("Message class is None, trying fallback by sensor type")
        common_types = {
            "camera": "sensor_msgs/Image",
            "image": "sensor_msgs/Image",
            "laser": "sensor_msgs/LaserScan",
            "scan": "sensor_msgs/LaserScan",
            "imu": "sensor_msgs/Imu",
            "contact": "gazebo_msgs/ContactsState",
            "bumper": "gazebo_msgs/ContactsState",
            "pointcloud": "sensor_msgs/PointCloud2",
            "depth": "sensor_msgs/Image",
            "sonar": "sensor_msgs/Range",
            "gps": "sensor_msgs/NavSatFix",
            "joint": "sensor_msgs/JointState",
            "temperature": "sensor_msgs/Temperature",
            "fluid": "sensor_msgs/FluidPressure",
            "magnetic": "sensor_msgs/MagneticField",
        }

        sensor_type = getattr(sensor, "sensor_type", "").lower()
        logger.debug("Sensor type: %s", sensor_type)
        msg_type_name = common_types.get(sensor_type)
        logger.debug("Fallback message type: %s", msg_type_name)

        if msg_type_name:
            try:
                package, msg_name = msg_type_name.split("/")
                logger.debug("Importing %s.msg.%s", package, msg_name)
                module = __import__(f"{package}.msg", fromlist=[msg_name])
                msg_class = getattr(module, msg_name)
                logger.debug("Successfully imported: %s", msg_class)
            except Exception as e:
                logger.debug("Failed to import: %s", e)
                pass

    if msg_class is None:
        result["error"] = "Could not determine message type"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Could not determine message type")
        return result

    result["message_type"] = msg_class.__name__
    logger.debug("Using message class: %s", msg_class.__name__)

    if progress_cb:
        try:
            progress_cb(60)
            logger.debug("Progress callback 60%%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    logger.info("Capturing from %s ...", sensor.topic)
    try:
        msgs = sensor.capture_data(
            msg_class, topic=sensor.topic, window=3.0, timeout=1.0, simulator=simulator
        )
        if not msgs:
            result["error"] = "No messages received within capture window"
            logger.error("No messages received on %s", sensor.topic)
        else:
            msg = msgs[-1]
            logger.info(
                "Captured %d message(s), type=%s", len(msgs), type(msg).__name__
            )
            result["data_received"] = True
            result["passed"] = True
            result["frames_received"] = len(msgs)

            if hasattr(msg, "header"):
                result["frame_id"] = msg.header.frame_id
            if hasattr(msg, "states"):
                result["contact_count"] = len(msg.states)
                if msg.states:
                    result["first_contact"] = {
                        "collision1": msg.states[0].collision1_name,
                        "collision2": msg.states[0].collision2_name,
                    }
            elif hasattr(msg, "height") and hasattr(msg, "data"):
                result["width"] = msg.width
                result["height"] = msg.height
                result["encoding"] = msg.encoding
            elif hasattr(msg, "ranges"):
                result["num_ranges"] = len(msg.ranges)
                result["angle_min"] = msg.angle_min
                result["angle_max"] = msg.angle_max
            elif hasattr(msg, "angular_velocity"):
                result["angular_velocity"] = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ]
                result["linear_acceleration"] = [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
    except Exception as e:
        result["error"] = str(e)
        logger.error("Capture error on %s: %s", sensor.topic, e)
        import traceback

        traceback.print_exc()

    if progress_cb:
        try:
            progress_cb(100)
            logger.debug("Progress callback 100%%")
        except:
            pass

    result["duration"] = round(time.time() - t0, 2)
    logger.debug("Test completed in %.2fs", result["duration"])
    logger.debug("Result: %s", "PASSED" if result["passed"] else "FAILED")
    if result.get("error"):
        logger.debug("Error: %s", result["error"])
    logger.info("=" * 60)
    logger.info("")  # Empty line for spacing

    return result


# ── Camera debug / type-check ─────────────────────────────────────────────────


def camera_check(simulator, sensor, progress_cb=None) -> dict:
    """
    Диагностическая проверка камеры: определяет тип (mono/depth/stereo),
    подписывается на топики, получает кадр и преобразует в стандартный BGR.
    """
    t0 = time.time()
    result: Dict[str, Any] = {
        "passed": False,
        "camera_type": "unknown",
        "topics_checked": [],
        "topics_active": [],
        "resolution": None,
        "encoding": None,
        "frame_bgr": None,
        "frame_shape": None,
        "error": None,
    }

    sdf_path = str(getattr(sensor, "sdf_path", "") or "")
    camera_type = "unknown"
    if sdf_path:
        try:
            camera_type = _camera_classify_sensor_profile(sdf_path)
        except Exception as e:
            logger.warning("camera_check: classify failed: %s", e)

    result["camera_type"] = camera_type
    logger.info("camera_check: type=%s sdf=%s", camera_type, sdf_path)

    world_path = _world_from_db(sensor, "camera_check")
    if not world_path:
        worlds_root = _camera_worlds_root()
        world_path = str(worlds_root / "camera_c1_single_cube.world")
        logger.debug("Using default world path: %s", world_path)

    if not os.path.exists(world_path):
        result["error"] = f"World file not found: {world_path}"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("World file not found: %s", world_path)
        return result

    logger.info("Opening Gazebo scene with world: %s", world_path)
    if not simulator.open_scene(world_path, sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Failed to open Gazebo scene")
        return result

    logger.debug("Gazebo scene opened successfully")

    if progress_cb:
        try:
            progress_cb(30)
            logger.debug("Progress callback 30%%")
        except Exception:
            pass

    logger.debug("Waiting 3 seconds for plugins to initialize...")
    time.sleep(3.0)

    try:
        published = rospy.get_published_topics()
        logger.debug("Found %d published topics", len(published))
    except Exception as e:
        logger.warning("Failed to get published topics: %s", e)
        published = []

    all_topic_names = [t for t, _ in published]
    logger.debug("All topics: %s", all_topic_names[:20])

    topics_to_check: List[str] = []

    if camera_type == "stereo":
        profile = _camera_load_sensor_profile(sdf_path) if sdf_path else {}
        left_t = str(profile.get("left_topic", "") or getattr(sensor, "topic", ""))
        right_t = str(profile.get("right_topic", ""))
        if left_t:
            topics_to_check.append(left_t)
        if right_t:
            topics_to_check.append(right_t)
        logger.debug("Stereo camera: left=%s, right=%s", left_t, right_t)
    elif camera_type == "depth":
        profile = _camera_load_sensor_profile(sdf_path) if sdf_path else {}
        image_t = str(profile.get("image_topic", "") or getattr(sensor, "topic", ""))
        depth_t = str(profile.get("depth_topic", ""))
        if image_t:
            topics_to_check.append(image_t)
        if depth_t:
            topics_to_check.append(depth_t)
        logger.debug("Depth camera: image=%s, depth=%s", image_t, depth_t)
    else:
        primary = str(getattr(sensor, "topic", ""))
        if primary:
            topics_to_check.append(primary)
        logger.debug("Mono camera: topic=%s", primary)

    result["topics_checked"] = list(topics_to_check)
    result["topics_active"] = [t for t in topics_to_check if t in all_topic_names]
    logger.debug("Topics checked: %s", topics_to_check)
    logger.debug("Topics active: %s", result["topics_active"])

    if progress_cb:
        try:
            progress_cb(50)
            logger.debug("Progress callback 50%%")
        except Exception:
            pass

    primary_topic = topics_to_check[0] if topics_to_check else ""
    if not primary_topic:
        result["error"] = "No topic configured for this sensor"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("No topic configured for sensor")
        return result

    if primary_topic not in all_topic_names:
        result["error"] = f"Topic {primary_topic} not published"
        result["available_topics"] = [
            t for t in all_topic_names if not t.startswith("/rosout")
        ][:20]
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Topic %s not published", primary_topic)
        return result

    logger.info("Waiting for image on %s (timeout=10s)...", primary_topic)
    try:
        msgs = sensor.capture_data(
            Image, topic=primary_topic, window=3.0, timeout=0.5, simulator=simulator,
        )
        if not msgs:
            raise RuntimeError(f"No messages received on {primary_topic}")
        msg = msgs[-1]
        logger.info(
            "Image received! %dx%d, encoding=%s", msg.width, msg.height, msg.encoding
        )
    except Exception as e:
        result["error"] = f"Timeout waiting for Image on {primary_topic}: {e}"
        result["duration"] = round(time.time() - t0, 2)
        logger.error("Failed to receive image: %s", e)
        return result

    if progress_cb:
        try:
            progress_cb(80)
            logger.debug("Progress callback 80%%")
        except Exception:
            pass

    result["encoding"] = msg.encoding
    result["resolution"] = f"{msg.width}x{msg.height}"

    try:
        bgr = _camera_check_to_bgr(msg)
        result["frame_bgr"] = bgr
        result["frame_shape"] = list(bgr.shape)
        result["passed"] = True
        logger.debug("Successfully converted frame to BGR, shape: %s", bgr.shape)
    except Exception as e:
        result["error"] = f"Failed to convert frame to BGR: {e}"
        logger.error("Frame conversion failed: %s", e)

    if progress_cb:
        try:
            progress_cb(100)
            logger.debug("Progress callback 100%%")
        except Exception:
            pass

    result["duration"] = round(time.time() - t0, 2)
    logger.info(
        "camera_check: type=%s passed=%s resolution=%s encoding=%s topics_active=%s duration=%.2fs",
        result["camera_type"],
        result["passed"],
        result["resolution"],
        result["encoding"],
        result["topics_active"],
        result["duration"],
    )
    return result


def _camera_check_to_bgr(msg: Image) -> np.ndarray:
    """Преобразует ROS Image в BGR numpy array, поддерживая основные кодировки."""
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()
    logger.debug("Converting image: %dx%d, encoding=%s", w, h, enc)

    if enc in ("rgb8", "r8g8b8"):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if enc == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3).copy()

    if enc in ("mono8", "8uc1"):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if enc == "32fc1":
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = depth[np.isfinite(depth)]
        if valid.size > 0:
            d_min, d_max = float(valid.min()), float(valid.max())
            if d_max > d_min:
                norm = (
                    ((depth - d_min) / (d_max - d_min) * 255)
                    .clip(0, 255)
                    .astype(np.uint8)
                )
            else:
                norm = np.zeros((h, w), dtype=np.uint8)
        else:
            norm = np.zeros((h, w), dtype=np.uint8)
        logger.debug(
            "Depth 32fc1: range [%.3f, %.3f]",
            d_min if valid.size > 0 else 0,
            d_max if valid.size > 0 else 0,
        )
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)

    if enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        depth_m = raw.astype(np.float32) / 1000.0
        valid = depth_m[depth_m > 0]
        if valid.size > 0:
            d_min, d_max = float(valid.min()), float(valid.max())
            if d_max > d_min:
                norm = (
                    ((depth_m - d_min) / (d_max - d_min) * 255)
                    .clip(0, 255)
                    .astype(np.uint8)
                )
            else:
                norm = np.zeros((h, w), dtype=np.uint8)
        else:
            norm = np.zeros((h, w), dtype=np.uint8)
        logger.debug(
            "Depth 16uc1: range [%.3f, %.3f] mm",
            raw.min() if raw.size > 0 else 0,
            raw.max() if raw.size > 0 else 0,
        )
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)

    logger.error("Unsupported encoding for camera_check: %s", enc)
    raise ValueError(f"Unsupported encoding for camera_check: {enc}")
