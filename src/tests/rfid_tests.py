"""RFID sensor tests."""

import math
import threading
import time

from ._common import _PoseStamped, Worlds


def rfid_max_stable_read_distance(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    import os

    read_distance = float(sensor.params.get("rzero", 3))

    map_path = CONFIG["RFID_MAP_PATH"]
    print(f"[DEBUG rfid_max_distance] rzero={read_distance}, map={os.path.abspath(map_path)}, sdf={sensor.sdf_path}")

    with open(map_path, "w") as f:
        f.write("fix1 1 0.5 0 0\n")

    if not simulator.open_scene(Worlds.RFID_CHANGE_DISTANCE.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    print(f"[DEBUG rfid_max_distance] scene opened")

    tag = "rfid_tag1"
    if not simulator.wait_for_model_spawn(tag, 30):
        raise RuntimeError("tag not spawned")

    current = 0.5
    reset = read_distance * 5
    max_dist = 0.0
    steps = max(1, round((read_distance - 0.5) / 0.5) + 1)
    step = 0
    t0 = time.time()

    while current <= read_distance + 1e-9:
        try:
            simulator.set_pose(tag, reset, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag, current, 0, 0)
            ros_msgs = sensor.capture_data(
                _PoseStamped(), window=3, simulator=simulator
            )
            rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}

            if tag in rfid_tags:
                max_dist = current

        except Exception:
            pass
        step += 1
        if progress_cb:
            progress_cb(int(step / steps * 100))
        current = round(current + 0.5, 1)

    return {
        "passed": abs(max_dist - read_distance) <= 0.5,
        "description": f"Максимальный радиус считывания по итогам теста: {max_dist:.2f}. Допустимо отклонение ±0.5м от заданного значения {read_distance:.2f}м.",
        "duration": time.time() - t0,
        "max_read_distance": max_dist,
    }


def rfid_min_stable_read_distance(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG

    with open(CONFIG["RFID_MAP_PATH"], "w") as f:
        f.write("fix1 1 0.5 0 0\n")

    if not simulator.open_scene(Worlds.RFID_CHANGE_DISTANCE.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")

    tag = "rfid_tag1"
    if not simulator.wait_for_model_spawn(tag, 30):
        raise RuntimeError("tag not spawned")

    min_dist = current = 0.25
    reset = 25.0
    steps = max(1, round(0.25 / 0.01))
    step = 0
    t0 = time.time()

    while current >= 0:
        try:
            simulator.set_pose(tag, reset, 0, 0)
            time.sleep(0.005)
            simulator.set_pose(tag, current, 0, 0)
            ros_msgs = sensor.capture_data(
                _PoseStamped(), window=3, simulator=simulator
            )
            rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}
            if tag in rfid_tags:
                min_dist = current
        except Exception:
            pass
        step += 1
        if progress_cb:
            progress_cb(int(step / steps * 100))
        current = round(current - 0.01, 5)

    return {
        "passed": min_dist <= 0.05,
        "description": f"Минимальный радиус считывания по итогам теста: {min_dist:.2f}. Допустимо значение не более 0.05м",
        "duration": time.time() - t0,
        "min_read_distance": min_dist,
    }


def rfid_mass_read(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    import os

    read_distance = float(sensor.params.get("rzero", 3))
    tags_count = 10
    radius = read_distance / 2

    map_path = CONFIG["RFID_MAP_PATH"]
    abs_map_path = os.path.abspath(map_path)
    print(f"[DEBUG rfid_mass_read] rzero={read_distance}, radius={radius}, tags_count={tags_count}")
    print(f"[DEBUG rfid_mass_read] map_path={map_path}")
    print(f"[DEBUG rfid_mass_read] abs_map_path={abs_map_path}")

    with open(map_path, "w") as f:
        for i in range(tags_count):
            x = radius * math.cos(2 * math.pi / tags_count * i)
            y = radius * math.sin(2 * math.pi / tags_count * i)
            line = f"fix{i+1} {i+1} {x} {y} 0\n"
            f.write(line)
            if i < 3:
                print(f"[DEBUG rfid_mass_read] map line: {line.strip()}")

    # Verify map was written
    with open(map_path, "r") as f:
        map_lines = f.readlines()
    print(f"[DEBUG rfid_mass_read] map written: {len(map_lines)} lines")

    world_path = Worlds.RFID_MASS_READ.value
    sdf_path = sensor.sdf_path
    print(f"[DEBUG rfid_mass_read] world={world_path}, sdf={sdf_path}")
    print(f"[DEBUG rfid_mass_read] world exists={os.path.exists(world_path)}, sdf exists={os.path.exists(sdf_path)}")

    if not simulator.open_scene(world_path, sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    print(f"[DEBUG rfid_mass_read] scene opened OK")

    # Check which models actually spawned
    spawned = []
    not_spawned = []
    for i in range(tags_count):
        tag_name = f"rfid_tag{i+1}"
        ok = simulator.wait_for_model_spawn(tag_name, 30)
        if ok:
            spawned.append(tag_name)
        else:
            not_spawned.append(tag_name)
        if progress_cb:
            progress_cb(int((i + 1) / tags_count * 50))

    print(f"[DEBUG rfid_mass_read] spawned: {len(spawned)}/{tags_count} — {spawned}")
    if not_spawned:
        print(f"[DEBUG rfid_mass_read] NOT spawned: {not_spawned}")

    if not spawned:
        raise RuntimeError(f"No tags spawned at all. Map path: {abs_map_path}")

    # List all models in Gazebo
    try:
        import rospy
        from gazebo_msgs.srv import GetWorldProperties
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5)
        get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        world_props = get_world()
        all_models = world_props.model_names
        rfid_models = [m for m in all_models if "rfid" in m.lower() or "tag" in m.lower() or "fix" in m.lower()]
        print(f"[DEBUG rfid_mass_read] all models in Gazebo ({len(all_models)}): {all_models}")
        print(f"[DEBUG rfid_mass_read] rfid-related models: {rfid_models}")
    except Exception as e:
        print(f"[DEBUG rfid_mass_read] could not list models: {e}")

    t0 = time.time()
    print(f"[DEBUG rfid_mass_read] starting capture_data on topic={sensor.topic}, window=20s")
    ros_msgs = sensor.capture_data(_PoseStamped(), window=20, simulator=simulator)
    print(f"[DEBUG rfid_mass_read] capture done: {len(ros_msgs)} raw messages")

    # Log all unique frame_ids
    all_frame_ids = [msg.header.frame_id for msg in ros_msgs]
    unique_ids = set(all_frame_ids)
    print(f"[DEBUG rfid_mass_read] unique frame_ids: {unique_ids}")
    print(f"[DEBUG rfid_mass_read] frame_id counts: {dict((fid, all_frame_ids.count(fid)) for fid in unique_ids)}")

    rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}
    print(f"[DEBUG rfid_mass_read] detected tags (unique): {list(rfid_tags.keys())}")

    if progress_cb:
        progress_cb(100)

    return {
        "passed": len(rfid_tags) / tags_count >= 0.75,
        "description": (
            f"Количество считанных тегов по итогам теста: {len(rfid_tags)}. "
            f"Допустимо значение не менее 75% от общего количества тегов: {tags_count}. "
            f"Spawned: {len(spawned)}/{tags_count}. Raw msgs: {len(ros_msgs)}. "
            f"Unique frame_ids: {list(rfid_tags.keys())}"
        ),
        "duration": time.time() - t0,
        "tags_detected_count": len(rfid_tags),
        "tags_spawned": len(spawned),
        "tags_not_spawned": not_spawned,
        "unique_frame_ids": list(rfid_tags.keys()),
        "raw_msg_count": len(ros_msgs),
    }


def rfid_overlap_tags(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    import os

    read_distance = float(sensor.params.get("rzero", 3))
    tags_count = 5
    radius = read_distance / 2
    distances = [0.2, 0.1, 0.05, 0.02]
    dist_result = None
    t0 = time.time()
    per_step_results = []

    map_path = CONFIG["RFID_MAP_PATH"]
    print(f"[DEBUG rfid_overlap_tags] rzero={read_distance}, radius={radius}, tags_count={tags_count}")
    print(f"[DEBUG rfid_overlap_tags] map_path={os.path.abspath(map_path)}")

    for step, distance in enumerate(distances):
        print(f"[DEBUG rfid_overlap_tags] --- step {step}: distance={distance} ---")
        with open(map_path, "w") as f:
            for i in range(tags_count):
                x = radius * math.cos(distance / radius * i)
                y = radius * math.sin(distance / radius * i)
                line = f"fix{i+1} {i+1} {x} {y} 0\n"
                f.write(line)

        # Verify map
        with open(map_path, "r") as f:
            map_lines = f.readlines()
        print(f"[DEBUG rfid_overlap_tags] map written: {len(map_lines)} lines")

        if not simulator.open_scene(Worlds.RFID_OVERLAP_TAGS.value, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        print(f"[DEBUG rfid_overlap_tags] scene opened")

        spawned = 0
        for i in range(tags_count):
            if simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
                spawned += 1
        print(f"[DEBUG rfid_overlap_tags] spawned: {spawned}/{tags_count}")

        ros_msgs = sensor.capture_data(_PoseStamped(), window=5, simulator=simulator)
        rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}
        print(f"[DEBUG rfid_overlap_tags] raw msgs: {len(ros_msgs)}, unique tags: {list(rfid_tags.keys())}")

        per_step_results.append({
            "distance": distance,
            "spawned": spawned,
            "raw_msgs": len(ros_msgs),
            "detected": list(rfid_tags.keys()),
        })

        if len(rfid_tags) >= tags_count * 0.5:
            dist_result = distance
        if progress_cb:
            progress_cb(int((step + 1) / len(distances) * 100))

    return {
        "passed": dist_result is not None,
        "description": (
            f"Расстояние между тегами по итогам теста: {dist_result}м. "
            f"Допустимое значение не более {distances[0]}м"
        ),
        "duration": time.time() - t0,
        "dist_result": dist_result,
        "per_step_results": per_step_results,
    }


def rfid_angle_dependence(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    import os

    read_distance = float(sensor.params.get("rzero", 3))
    angles = [0, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2]
    radius = read_distance / 2

    map_path = CONFIG["RFID_MAP_PATH"]
    print(f"[DEBUG rfid_angle_dependence] rzero={read_distance}, radius={radius}, angles={len(angles)}")
    print(f"[DEBUG rfid_angle_dependence] map_path={os.path.abspath(map_path)}")

    with open(map_path, "w") as f:
        for i, angle in enumerate(angles):
            x = radius * math.sin(angle)
            z = radius * math.cos(angle)
            line = f"fix{i+1} {i+1} {x} 0 {z}\n"
            f.write(line)
            print(f"[DEBUG rfid_angle_dependence] map: {line.strip()}")

    world_path = Worlds.RFID_ANGLE_DEPENDENCE.value
    print(f"[DEBUG rfid_angle_dependence] world={world_path}, sdf={sensor.sdf_path}")

    if not simulator.open_scene(world_path, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    print(f"[DEBUG rfid_angle_dependence] scene opened OK")

    spawned = []
    not_spawned = []
    for i in range(len(angles)):
        tag_name = f"rfid_tag{i+1}"
        ok = simulator.wait_for_model_spawn(tag_name, 30)
        if ok:
            spawned.append(tag_name)
        else:
            not_spawned.append(tag_name)
        if progress_cb:
            progress_cb(int((i + 1) / len(angles) * 50))

    print(f"[DEBUG rfid_angle_dependence] spawned: {len(spawned)}/{len(angles)} — {spawned}")
    if not_spawned:
        print(f"[DEBUG rfid_angle_dependence] NOT spawned: {not_spawned}")

    # List Gazebo models
    try:
        import rospy
        from gazebo_msgs.srv import GetWorldProperties
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5)
        get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        all_models = get_world().model_names
        print(f"[DEBUG rfid_angle_dependence] Gazebo models: {all_models}")
    except Exception as e:
        print(f"[DEBUG rfid_angle_dependence] could not list models: {e}")

    t0 = time.time()
    print(f"[DEBUG rfid_angle_dependence] capturing on topic={sensor.topic}, window=20s")
    ros_msgs = sensor.capture_data(_PoseStamped(), window=20, simulator=simulator)
    print(f"[DEBUG rfid_angle_dependence] raw msgs: {len(ros_msgs)}")

    all_frame_ids = [msg.header.frame_id for msg in ros_msgs]
    unique_ids = set(all_frame_ids)
    print(f"[DEBUG rfid_angle_dependence] unique frame_ids: {unique_ids}")
    print(f"[DEBUG rfid_angle_dependence] frame_id counts: {dict((fid, all_frame_ids.count(fid)) for fid in unique_ids)}")

    rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}
    if progress_cb:
        progress_cb(100)

    return {
        "passed": len(rfid_tags) == len(angles),
        "description": (
            f"Количество считанных тегов по итогам теста: {len(rfid_tags)}. "
            f"Допустимо значение должно быть равно: {len(angles)}. "
            f"Spawned: {len(spawned)}/{len(angles)}. Raw msgs: {len(ros_msgs)}. "
            f"Unique frame_ids: {list(rfid_tags.keys())}"
        ),
        "duration": time.time() - t0,
        "tags_detected_count": len(rfid_tags),
        "tags_spawned": len(spawned),
        "unique_frame_ids": list(rfid_tags.keys()),
    }


def rfid_move_tags(simulator, sensor, progress_cb=None) -> dict:
    """Перемещение тега только через set_pose; «скорость» — шаг/пауза между позами.

    capture_data крутится в отдельном потоке, пока основной поток двигает модель.
    """
    from config import CONFIG

    read_distance = float(sensor.params.get("rzero", 3))
    factor = 1.5
    start_dist = -read_distance * factor
    end_x = read_distance * factor
    path_length = end_x - start_dist
    # Номинальные скорости (м/с): сначала медленнее — выше шанс стабильного чтения одного тега
    speeds_m_s = [0.2, 0.4, 0.7, 1.0]
    step_m = 0.05
    min_dt = 0.012

    result_speed = None
    t0 = time.time()

    n_steps = max(1, int(math.ceil(path_length / step_m)))
    dx = path_length / n_steps

    for step, v in enumerate(speeds_m_s):
        with open(CONFIG["RFID_MAP_PATH"], "w") as f:
            f.write(f"fix1 1 {start_dist} 0 0\n")
        if not simulator.open_scene(Worlds.RFID_MOVE_TAGS.value, sensor.sdf_path):
            raise RuntimeError("failed to open Gazebo scene")
        if not simulator.wait_for_model_spawn("rfid_tag1", 30):
            raise RuntimeError("tag not spawned")

        simulator.set_pose("rfid_tag1", x=start_dist, y=0, z=0)
        time.sleep(0.02)

        dt_step = max(min_dt, dx / max(v, 0.05))
        sweep_time = n_steps * dt_step
        capture_window = sweep_time + 5.0

        ros_msgs_holder: list = []

        def _capture_worker():
            ros_msgs_holder.extend(
                sensor.capture_data(
                    _PoseStamped(),
                    window=capture_window,
                    simulator=simulator,
                )
            )

        th = threading.Thread(target=_capture_worker, daemon=True)
        th.start()
        time.sleep(0.06)

        x = start_dist
        for _ in range(n_steps):
            x += dx
            simulator.set_pose("rfid_tag1", x=x, y=0, z=0)
            time.sleep(dt_step)

        th.join()

        rfid_tags = {
            msg.header.frame_id: msg.pose
            for msg in ros_msgs_holder
            if hasattr(msg, "header") and hasattr(msg, "pose")
        }
        if len(rfid_tags) == 1:
            result_speed = v if result_speed is None else max(result_speed, v)

        if progress_cb:
            progress_cb(int((step + 1) / len(speeds_m_s) * 100))

    return {
        "passed": result_speed is not None,
        "description": f"По итогам теста максимальная скорость метки, при которой она считывается: {result_speed} м/с. Допустимое значение не менее {speeds_m_s[0]} м/с",
        "duration": time.time() - t0,
        "max_detected_velocity": result_speed,
    }


def rfid_antenna_rotation(simulator, sensor, progress_cb=None) -> dict:
    from config import CONFIG
    from geometry_msgs.msg import Quaternion
    import os

    read_distance = float(sensor.params.get("rzero", 3))
    angles = [0, math.pi / 6, math.pi / 3, math.pi / 2]
    tags_count = 10
    radius = read_distance / 2
    passed = False
    angle2count = {}

    map_path = CONFIG["RFID_MAP_PATH"]
    print(f"[DEBUG rfid_antenna_rotation] rzero={read_distance}, radius={radius}, tags_count={tags_count}")
    print(f"[DEBUG rfid_antenna_rotation] map_path={os.path.abspath(map_path)}")

    with open(map_path, "w") as f:
        for i in range(tags_count):
            x = radius * math.cos(2 * math.pi / tags_count * i)
            y = radius * math.sin(2 * math.pi / tags_count * i)
            f.write(f"fix{i+1} {i+1} {x} {y} 0\n")

    with open(map_path, "r") as f:
        print(f"[DEBUG rfid_antenna_rotation] map written: {len(f.readlines())} lines")

    if not simulator.open_scene(Worlds.RFID_ANTENNA_ROTATION.value, sensor.sdf_path):
        raise RuntimeError("failed to open Gazebo scene")
    print(f"[DEBUG rfid_antenna_rotation] scene opened")

    spawned = 0
    for i in range(tags_count):
        if simulator.wait_for_model_spawn(f"rfid_tag{i+1}", 30):
            spawned += 1
    print(f"[DEBUG rfid_antenna_rotation] spawned: {spawned}/{tags_count}")

    # List Gazebo models
    try:
        import rospy
        from gazebo_msgs.srv import GetWorldProperties
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=5)
        all_models = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)().model_names
        print(f"[DEBUG rfid_antenna_rotation] Gazebo models: {all_models}")
    except Exception as e:
        print(f"[DEBUG rfid_antenna_rotation] could not list models: {e}")

    t0 = time.time()
    for step, angle in enumerate(angles):
        q = Quaternion(0, 0, math.sin(angle / 2), math.cos(angle / 2))
        simulator.set_pose("rfid_antenna", x=0, y=0, z=0, quaternion=q)
        time.sleep(0.01)

        print(f"[DEBUG rfid_antenna_rotation] angle={math.degrees(angle):.1f}° capturing...")
        ros_msgs = sensor.capture_data(_PoseStamped(), window=7, simulator=simulator)
        rfid_tags = {msg.header.frame_id: msg.pose for msg in ros_msgs}
        print(f"[DEBUG rfid_antenna_rotation] angle={math.degrees(angle):.1f}°: raw={len(ros_msgs)}, unique={list(rfid_tags.keys())}")

        if len(rfid_tags) >= tags_count * 0.5 and angle >= 0:
            passed = True
        angle2count[round(math.degrees(angle), 1)] = len(rfid_tags)
        if progress_cb:
            progress_cb(int((step + 1) / len(angles) * 100))

    return {
        "passed": passed,
        "description": (
            f"Угол поворота антенны, при котором считываются метки. "
            f"Spawned: {spawned}/{tags_count}. Результаты по углам: {angle2count}"
        ),
        "duration": time.time() - t0,
        "angle2tags_count": angle2count,
        "tags_spawned": spawned,
    }
