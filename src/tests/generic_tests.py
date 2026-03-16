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


# ── Camera smoke / generic tests ─────────────────────────────────────────────


def camera_data_received(simulator, sensor, progress_cb=None) -> dict:
    """
    Smoke test: verify the camera publishes a valid image.
    Pass criterion: at least one non-empty frame received.
    """
    world = _world_from_db(sensor, "camera_data_received") or Worlds.CAMERA_SMOKE.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    if progress_cb:
        progress_cb(20)

    t0   = time.time()
    msgs = sensor.capture_data(Image, window=15.0, timeout=2.0, warmup=3.0, simulator=simulator)

    if not msgs:
        return {
            "passed":   False,
            "duration": round(time.time() - t0, 2),
            "detail":   "No image received within 15 s",
        }

    arr       = _img_to_numpy(msgs[0])
    non_zero  = int(np.count_nonzero(arr))

    if progress_cb:
        progress_cb(100)

    return {
        "passed":      non_zero > 0,
        "duration":    round(time.time() - t0, 2),
        "resolution":  f"{msgs[0].width}x{msgs[0].height}",
        "encoding":    msgs[0].encoding,
        "frames_recv": len(msgs),
        "non_zero_px": non_zero,
    }


def camera_depth_accuracy(simulator, sensor, progress_cb=None) -> dict:
    """
    Method C1 — Depth Accuracy.
    Pass criterion: relative error δ ≤ 2 % at Z_true = 3.0 m.
    """
    Z_TRUE    = 3.0
    PASS_PCT  = 2.0

    world = _world_from_db(sensor, "camera_depth_accuracy") or Worlds.CAMERA_DEPTH_ACCURACY.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    if progress_cb:
        progress_cb(20)

    t0   = time.time()
    msgs = sensor.capture_data(Image, window=5.0, timeout=2.0, warmup=3.0, simulator=simulator)

    if not msgs:
        raise RuntimeError("No depth frames received")

    if progress_cb:
        progress_cb(60)

    frames     = [_img_to_numpy(m)[:, :, 0] for m in msgs]
    depth_map  = np.nanmedian(np.stack(frames, axis=0), axis=0)
    cy, cx     = depth_map.shape[0] // 2, depth_map.shape[1] // 2
    patch      = depth_map[cy - 5:cy + 5, cx - 5:cx + 5]
    z_measured = float(np.nanmedian(patch))
    abs_err    = abs(z_measured - Z_TRUE)
    rel_err    = abs_err / Z_TRUE * 100.0

    if progress_cb:
        progress_cb(100)

    return {
        "passed":        rel_err <= PASS_PCT,
        "duration":      round(time.time() - t0, 2),
        "z_true_m":      Z_TRUE,
        "z_measured_m":  round(z_measured, 4),
        "abs_error_m":   round(abs_err, 4),
        "rel_error_pct": round(rel_err, 2),
        "frames_used":   len(msgs),
    }


def camera_resolution(simulator, sensor, progress_cb=None) -> dict:
    """
    Method C2 — Resolution.
    Pass criterion: two contours distinguishable at separation ≥ 0.05 m.
    """
    SEPARATIONS  = [0.20, 0.10, 0.05, 0.02]
    PASS_MIN_SEP = 0.05
    SPHERE_A     = "sphere_a"
    SPHERE_B     = "sphere_b"

    world = _world_from_db(sensor, "camera_resolution") or Worlds.CAMERA_RESOLUTION.value
    if not simulator.open_scene(world, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    if not simulator.wait_for_model_spawn(SPHERE_A, 30):
        raise RuntimeError(f"{SPHERE_A} did not spawn")
    if not simulator.wait_for_model_spawn(SPHERE_B, 30):
        raise RuntimeError(f"{SPHERE_B} did not spawn")

    if progress_cb:
        progress_cb(20)

    t0          = time.time()
    min_sep_ok  = None
    sep_results = {}

    for i, sep in enumerate(SEPARATIONS):
        half   = sep / 2.0
        warmup = 3.0 if i == 0 else 0.0
        simulator.set_pose(SPHERE_A, x=3.0, y=-half, z=0.0)
        simulator.set_pose(SPHERE_B, x=3.0, y= half, z=0.0)
        time.sleep(0.1)

        msgs = sensor.capture_data(Image, window=3.0, timeout=2.0, warmup=warmup, simulator=simulator)
        if not msgs:
            sep_results[f"{int(sep*1000)}mm"] = 0
            continue

        arr  = _img_to_numpy(msgs[-1])
        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY) if arr.shape[2] == 3 else arr[:, :, 0]
        _, bw = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        n = len([c for c in contours if cv2.contourArea(c) >= 50])
        sep_results[f"{int(sep*1000)}mm"] = n
        if n >= 2:
            min_sep_ok = sep

        if progress_cb:
            progress_cb(20 + int((i + 1) / len(SEPARATIONS) * 75))

    if progress_cb:
        progress_cb(100)

    return {
        "passed":           min_sep_ok is not None and min_sep_ok <= PASS_MIN_SEP,
        "duration":         round(time.time() - t0, 2),
        "min_sep_passed_m": min_sep_ok,
        "pass_threshold_m": PASS_MIN_SEP,
        "sep_contours":     sep_results,
    }


# ── Generic capture (any sensor) ─────────────────────────────────────────────


def sensor_capture_basic(simulator, sensor, progress_cb=None) -> dict:
    print(f"\n{'='*60}")
    print(f"TESTING SENSOR: {getattr(sensor, 'sensor_name', 'unknown')} ({getattr(sensor, 'sensor_type', 'unknown')})")
    print(f"TOPIC: {sensor.topic}")
    print(f"{'='*60}")

    result = {
        "passed": False,
        "topic": sensor.topic,
        "sensor_type": getattr(sensor, "sensor_type", "unknown"),
        "sensor_name": getattr(sensor, "sensor_name", "unknown"),
        "duration": 0,
        "message_type": None,
        "data_received": False,
        "error": None
    }

    t0 = time.time()

    world_path = "/home/alexey/Documents/projects/2042/github/2042_sensor_testing/resources/worlds/rfid/overlap_tags.world"
    print(f"[DEBUG] World path: {world_path}")
    print(f"[DEBUG] World exists: {os.path.exists(world_path)}")
    print(f"[DEBUG] Sensor SDF path: {sensor.sdf_path}")
    print(f"[DEBUG] Sensor SDF exists: {os.path.exists(sensor.sdf_path)}")

    if not os.path.exists(world_path):
        print(f"[WARN] World file not found, using fallback")
        fallback = "/tmp/default_world.world"
        with open(fallback, 'w') as f:
            f.write('''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
  </world>
</sdf>''')
        world_path = fallback
        print(f"[DEBUG] Created fallback world: {world_path}")

    print(f"[DEBUG] Opening Gazebo scene...")
    if not simulator.open_scene(world_path, sensor.sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Failed to open Gazebo scene")
        return result

    print(f"[DEBUG] Gazebo scene opened successfully")

    if progress_cb:
        try:
            progress_cb(20)
            print(f"[DEBUG] Progress callback 20%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    print(f"[DEBUG] Waiting 3 seconds for plugins to initialize...")
    time.sleep(3)

    print(f"[DEBUG] Getting published topics...")
    topics = rospy.get_published_topics()
    print(f"[DEBUG] Found {len(topics)} topics total")

    print(f"[DEBUG] Available topics:")
    for t, t_type in topics[:15]:
        if not t.startswith('/rosout'):
            print(f"        {t} -> {t_type}")

    topic_exists = any(t == sensor.topic for t, _ in topics)
    print(f"[DEBUG] Topic '{sensor.topic}' exists: {topic_exists}")

    if not topic_exists:
        result["error"] = f"Topic {sensor.topic} not found"
        result["available_topics"] = [t for t, _ in topics[:20] if not t.startswith('/rosout')]
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Topic not found!")
        return result

    if progress_cb:
        try:
            progress_cb(40)
            print(f"[DEBUG] Progress callback 40%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    print(f"[DEBUG] Getting message class for topic: {sensor.topic}")
    msg_class, real_topic, _ = rostopic.get_topic_class(sensor.topic)
    print(f"[DEBUG] rostopic.get_topic_class returned: {msg_class}")
    print(f"[DEBUG] Real topic: {real_topic}")

    if msg_class is None:
        print(f"[DEBUG] Message class is None, trying fallback by sensor type")
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
            "magnetic": "sensor_msgs/MagneticField"
        }

        sensor_type = getattr(sensor, "sensor_type", "").lower()
        print(f"[DEBUG] Sensor type: {sensor_type}")
        msg_type_name = common_types.get(sensor_type)
        print(f"[DEBUG] Fallback message type: {msg_type_name}")

        if msg_type_name:
            try:
                package, msg_name = msg_type_name.split('/')
                print(f"[DEBUG] Importing {package}.msg.{msg_name}")
                module = __import__(f"{package}.msg", fromlist=[msg_name])
                msg_class = getattr(module, msg_name)
                print(f"[DEBUG] Successfully imported: {msg_class}")
            except Exception as e:
                print(f"[DEBUG] Failed to import: {e}")
                pass

    if msg_class is None:
        result["error"] = "Could not determine message type"
        result["duration"] = round(time.time() - t0, 2)
        print(f"[ERROR] Could not determine message type")
        return result

    result["message_type"] = msg_class.__name__
    print(f"[DEBUG] Using message class: {msg_class.__name__}")

    if progress_cb:
        try:
            progress_cb(60)
            print(f"[DEBUG] Progress callback 60%")
        except:
            result["cancelled"] = True
            result["duration"] = round(time.time() - t0, 2)
            return result

    print(f"[DEBUG] Waiting for message on {sensor.topic} (timeout=10s)...")
    try:
        msg = rospy.wait_for_message(sensor.topic, msg_class, timeout=10.0)
        print(f"[DEBUG] Message received! Type: {type(msg).__name__}")
        result["data_received"] = True
        result["passed"] = True

        if hasattr(msg, 'header'):
            result["timestamp"] = msg.header.stamp.to_sec() if msg.header.stamp else None
            result["frame_id"] = msg.header.frame_id
            print(f"[DEBUG] Header: frame={msg.header.frame_id}, stamp={msg.header.stamp}")

        if hasattr(msg, 'states'):
            result["contact_count"] = len(msg.states)
            print(f"[DEBUG] Contact count: {len(msg.states)}")
            if msg.states:
                result["first_contact"] = {
                    "collision1": msg.states[0].collision1_name,
                    "collision2": msg.states[0].collision2_name
                }
                print(f"[DEBUG] First contact: {msg.states[0].collision1_name} -> {msg.states[0].collision2_name}")
        elif hasattr(msg, 'data') and hasattr(msg, 'height'):
            result["width"] = msg.width
            result["height"] = msg.height
            result["encoding"] = msg.encoding
            print(f"[DEBUG] Image: {msg.width}x{msg.height}, encoding={msg.encoding}")
            print(f"[DEBUG] Data length: {len(msg.data)} bytes")
        elif hasattr(msg, 'ranges'):
            result["num_ranges"] = len(msg.ranges)
            result["angle_min"] = msg.angle_min
            result["angle_max"] = msg.angle_max
            print(f"[DEBUG] Laser scan: {len(msg.ranges)} ranges")
        elif hasattr(msg, 'angular_velocity'):
            result["angular_velocity"] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            result["linear_acceleration"] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            print(f"[DEBUG] IMU data received")

    except rospy.ROSException as e:
        result["error"] = f"Timeout: No message received within 10s"
        print(f"[ERROR] Timeout: No message received on {sensor.topic}")
    except Exception as e:
        result["error"] = str(e)
        print(f"[ERROR] Exception: {e}")
        import traceback
        traceback.print_exc()

    if progress_cb:
        try:
            progress_cb(100)
            print(f"[DEBUG] Progress callback 100%")
        except:
            pass

    result["duration"] = round(time.time() - t0, 2)
    print(f"[DEBUG] Test completed in {result['duration']}s")
    print(f"[DEBUG] Result: {'PASSED' if result['passed'] else 'FAILED'}")
    if result.get('error'):
        print(f"[DEBUG] Error: {result['error']}")
    print(f"{'='*60}\n")

    if simulator:
        try:
            simulator.notify_capture({
                "sensor_type": result["sensor_type"],
                "sensor_name": result["sensor_name"],
                "topic": sensor.topic,
                "success": result["passed"]
            }, None)
        except:
            pass

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

    if not os.path.exists(world_path):
        result["error"] = f"World file not found: {world_path}"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if not simulator.open_scene(world_path, sdf_path):
        result["error"] = "Failed to open Gazebo scene"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        try:
            progress_cb(30)
        except Exception:
            pass

    time.sleep(3.0)

    try:
        published = rospy.get_published_topics()
    except Exception:
        published = []

    all_topic_names = [t for t, _ in published]

    topics_to_check: List[str] = []

    if camera_type == "stereo":
        profile = _camera_load_sensor_profile(sdf_path) if sdf_path else {}
        left_t = str(profile.get("left_topic", "") or getattr(sensor, "topic", ""))
        right_t = str(profile.get("right_topic", ""))
        if left_t:
            topics_to_check.append(left_t)
        if right_t:
            topics_to_check.append(right_t)
    elif camera_type == "depth":
        profile = _camera_load_sensor_profile(sdf_path) if sdf_path else {}
        image_t = str(profile.get("image_topic", "") or getattr(sensor, "topic", ""))
        depth_t = str(profile.get("depth_topic", ""))
        if image_t:
            topics_to_check.append(image_t)
        if depth_t:
            topics_to_check.append(depth_t)
    else:
        primary = str(getattr(sensor, "topic", ""))
        if primary:
            topics_to_check.append(primary)

    result["topics_checked"] = list(topics_to_check)
    result["topics_active"] = [t for t in topics_to_check if t in all_topic_names]

    if progress_cb:
        try:
            progress_cb(50)
        except Exception:
            pass

    primary_topic = topics_to_check[0] if topics_to_check else ""
    if not primary_topic:
        result["error"] = "No topic configured for this sensor"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if primary_topic not in all_topic_names:
        result["error"] = f"Topic {primary_topic} not published"
        result["available_topics"] = [t for t in all_topic_names if not t.startswith("/rosout")][:20]
        result["duration"] = round(time.time() - t0, 2)
        return result

    try:
        msg = rospy.wait_for_message(primary_topic, Image, timeout=10.0)
    except Exception as e:
        result["error"] = f"Timeout waiting for Image on {primary_topic}: {e}"
        result["duration"] = round(time.time() - t0, 2)
        return result

    if progress_cb:
        try:
            progress_cb(80)
        except Exception:
            pass

    result["encoding"] = msg.encoding
    result["resolution"] = f"{msg.width}x{msg.height}"

    try:
        bgr = _camera_check_to_bgr(msg)
        result["frame_bgr"] = bgr
        result["frame_shape"] = list(bgr.shape)
        result["passed"] = True
    except Exception as e:
        result["error"] = f"Failed to convert frame to BGR: {e}"

    if progress_cb:
        try:
            progress_cb(100)
        except Exception:
            pass

    result["duration"] = round(time.time() - t0, 2)
    logger.info(
        "camera_check: type=%s passed=%s resolution=%s encoding=%s topics_active=%s",
        result["camera_type"], result["passed"], result["resolution"],
        result["encoding"], result["topics_active"],
    )
    return result


def _camera_check_to_bgr(msg: Image) -> np.ndarray:
    """Преобразует ROS Image в BGR numpy array, поддерживая основные кодировки."""
    h, w = int(msg.height), int(msg.width)
    enc = (msg.encoding or "").lower()

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
                norm = ((depth - d_min) / (d_max - d_min) * 255).clip(0, 255).astype(np.uint8)
            else:
                norm = np.zeros((h, w), dtype=np.uint8)
        else:
            norm = np.zeros((h, w), dtype=np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)

    if enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        depth_m = raw.astype(np.float32) / 1000.0
        valid = depth_m[depth_m > 0]
        if valid.size > 0:
            d_min, d_max = float(valid.min()), float(valid.max())
            if d_max > d_min:
                norm = ((depth_m - d_min) / (d_max - d_min) * 255).clip(0, 255).astype(np.uint8)
            else:
                norm = np.zeros((h, w), dtype=np.uint8)
        else:
            norm = np.zeros((h, w), dtype=np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)

    raise ValueError(f"Unsupported encoding for camera_check: {enc}")
