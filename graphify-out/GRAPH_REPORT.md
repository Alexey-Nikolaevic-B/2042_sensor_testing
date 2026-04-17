# Graph Report - .  (2026-04-16)

## Corpus Check
- 74 files · ~69,533 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 966 nodes · 2574 edges · 44 communities detected
- Extraction: 67% EXTRACTED · 33% INFERRED · 0% AMBIGUOUS · INFERRED: 840 edges (avg confidence: 0.71)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Sensor & Type Dialogs|Sensor & Type Dialogs]]
- [[_COMMUNITY_Mono Camera Tests|Mono Camera Tests]]
- [[_COMMUNITY_Test Queue & UI Items|Test Queue & UI Items]]
- [[_COMMUNITY_Sensor Storage & Lifecycle|Sensor Storage & Lifecycle]]
- [[_COMMUNITY_Sensor Repository & Main Window|Sensor Repository & Main Window]]
- [[_COMMUNITY_Config & Gazebo Process Control|Config & Gazebo Process Control]]
- [[_COMMUNITY_Depth Camera Tests|Depth Camera Tests]]
- [[_COMMUNITY_Stereo Camera Tests|Stereo Camera Tests]]
- [[_COMMUNITY_UI Theme & Styling|UI Theme & Styling]]
- [[_COMMUNITY_Core, Detector & Sensor Abstraction|Core, Detector & Sensor Abstraction]]
- [[_COMMUNITY_ROSCatkin Build & Project Docs|ROS/Catkin Build & Project Docs]]
- [[_COMMUNITY_Test Result Display Widget|Test Result Display Widget]]
- [[_COMMUNITY_RFID Tag Plugin (C++)|RFID Tag Plugin (C++)]]
- [[_COMMUNITY_RFID Antenna Plugin (C++)|RFID Antenna Plugin (C++)]]
- [[_COMMUNITY_Setup & Launch Documentation|Setup & Launch Documentation]]
- [[_COMMUNITY_SQLite DB Portability Notes|SQLite DB Portability Notes]]
- [[_COMMUNITY_Test Registration Convention|Test Registration Convention]]
- [[_COMMUNITY_Tactile SDF Rationale|Tactile SDF Rationale]]
- [[_COMMUNITY_Tag Header Definitions|Tag Header Definitions]]
- [[_COMMUNITY_Multi-plugin Detection Notes|Multi-plugin Detection Notes]]
- [[_COMMUNITY_Capture Data Patterns Notes|Capture Data Patterns Notes]]
- [[_COMMUNITY_Relative Path DB Convention|Relative Path DB Convention]]
- [[_COMMUNITY_Antenna Header|Antenna Header]]
- [[_COMMUNITY_Spinner Animation Helper|Spinner Animation Helper]]
- [[_COMMUNITY_Primary Topic Convention|Primary Topic Convention]]
- [[_COMMUNITY_Roscore Connectivity Probe|Roscore Connectivity Probe]]
- [[_COMMUNITY_ROS Node Init Probe|ROS Node Init Probe]]
- [[_COMMUNITY_Gazebo Liveness Probe|Gazebo Liveness Probe]]
- [[_COMMUNITY_UI Layout Helper|UI Layout Helper]]
- [[_COMMUNITY_Sensor Detector ABC|Sensor Detector ABC]]
- [[_COMMUNITY_rospkg Dependency|rospkg Dependency]]
- [[_COMMUNITY_empy Dependency|empy Dependency]]
- [[_COMMUNITY_PyYAML Dependency|PyYAML Dependency]]
- [[_COMMUNITY_numpy Dependency|numpy Dependency]]
- [[_COMMUNITY_PyQt5-sip Dependency|PyQt5-sip Dependency]]
- [[_COMMUNITY_opencv-python Dependency|opencv-python Dependency]]
- [[_COMMUNITY_defusedxml Dependency|defusedxml Dependency]]
- [[_COMMUNITY_Roscore Port Conflict Note|Roscore Port Conflict Note]]
- [[_COMMUNITY_Headless Camera Note|Headless Camera Note]]
- [[_COMMUNITY_Empty Result Dict Bug|Empty Result Dict Bug]]
- [[_COMMUNITY_Stereo Race Condition Bug|Stereo Race Condition Bug]]
- [[_COMMUNITY_Cooperative Stop Refactor|Cooperative Stop Refactor]]
- [[_COMMUNITY_gzserver Pkill Hazard|gzserver Pkill Hazard]]
- [[_COMMUNITY_SDF Camera Catalog|SDF Camera Catalog]]

## God Nodes (most connected - your core abstractions)
1. `Layout` - 58 edges
2. `ColSensors` - 44 edges
3. `ColTests` - 43 edges
4. `ColCapture` - 42 edges
5. `Icons` - 41 edges
6. `AddSensorDialog` - 39 edges
7. `TestItem` - 34 edges
8. `AddSensorTypeDialog` - 34 edges
9. `SensorRepository` - 32 edges
10. `_connect()` - 32 edges

## Surprising Connections (you probably didn't know these)
- `_draw_debug()` --calls--> `COPY()`  [INFERRED]
  src/tests/dcam_tests.py → front/_theme.py
- `__main__ entry point` --references--> `CONFIG singleton`  [INFERRED]
  __main__.py → config.py
- `front.Sensor wrapper` --semantically_similar_to--> `Sensor class`  [INFERRED] [semantically similar]
  front/logic_sensor_repository.py → src/sensor.py
- `Claude project context` --semantically_similar_to--> `Sensor Testing Gazebo/ROS App`  [INFERRED] [semantically similar]
  claude.md → README.md
- `AddSensorDialog` --uses--> `Sensor`  [INFERRED]
  front/dialog_add_sensor.py → src/sensor.py

## Hyperedges (group relationships)
- **Test execution pipeline: UI -> QueueManager -> TestRunner -> Worker -> UI result** — widget_col_3_coltests, logic_queue_manager_queuemanager, logic_test_runner_testrunner, logic_test_runner_worker, widget_col_4_colcapture [INFERRED 0.90]
- **Four-column UI layout: sensors, details, tests, capture/log** — widget_col_1_colsensors, widget_col_2_coldetails, widget_col_3_coltests, widget_col_4_colcapture [INFERRED 0.90]
- **Logging pipeline: Python logger -> _QtHandler -> _LogBridge signal -> ColCapture log_view** — logic_log_bridge_qthandler, logic_log_bridge_logbridge, logic_log_bridge_setup_logging, widget_col_4_colcapture [INFERRED 0.85]
- **Test registration pipeline: Core -> TESTS dict -> test functions** — core_core_get_tests, common_get_tests_for_type, common_tests_dict, sensor_storage_get_type_tests, sensor_storage_sensor_type_tests_table [INFERRED 0.90]
- **Sensor capture flow: capture_data -> notify_capture -> wait_for_step (step mode)** — sensor_sensor_capture_data, sensor_sensor_capture_persistent, gazebo_simulator_simulator_notify_capture, gazebo_simulator_simulator_wait_for_step, gazebo_simulator_simulator_capture_observer_frame [INFERRED 0.85]
- **Sensor type detection pipeline: SDF file -> detector -> DB-backed sensor type lookup** — detector_detect_sensor_type, detector_detect_from_db, detector_detector_registry, detector_custom_detectors, sensor_storage_get_all_sensor_types [INFERRED 0.90]
- **Catkin workspace packages built together** — scenario_test_pkg, rfid_antenna_plugin, rfid_tag_plugin, gazebo_to_ros_tf, cmake_toplevel_symlink [EXTRACTED 0.90]
- **RFID Gazebo simulation system (antenna+tag+tf)** — rfid_antenna_plugin, rfid_tag_plugin, gazebo_to_ros_tf, rfid_plugin_purpose, claudemd_sdf_summary_rfid [EXTRACTED 0.88]
- **Launch prerequisites (ROS+catkin+venv+python deps)** — readme_launch_sequence, readme_ros_noetic_install, readme_python314, requirements_pyqt5, requirements_numpy [EXTRACTED 0.85]

## Communities

### Community 0 - "Sensor & Type Dialogs"
Cohesion: 0.03
Nodes (60): Colors, LightColors, LightStyles, Styles, get_custom_detector_names(), _to_absolute, _to_relative, AddSensorDialog (+52 more)

### Community 1 - "Mono Camera Tests"
Cohesion: 0.05
Nodes (115): _camera_classify_sensor_profile(), _camera_load_sensor_profile(), _camera_worlds_root(), get_test(), _img_to_numpy(), _join_topic(), _normalize_topic(), _parse_pose() (+107 more)

### Community 2 - "Test Queue & UI Items"
Cohesion: 0.04
Nodes (28): __main__ entry point, Icons, get_tests_for_type(), Return {func_name: callable} for all tests registered for this sensor's type., Enum, _attach_all(), _attach_to_logger(), _LogBridge (+20 more)

### Community 3 - "Sensor Storage & Lifecycle"
Cohesion: 0.05
Nodes (82): Core class, _detect_from_db(), _RoscoreWatcher QThread, Simulator class, add_sensor(), _connect(), delete_sensor(), delete_sensor_type() (+74 more)

### Community 4 - "Sensor Repository & Main Window"
Cohesion: 0.04
Nodes (15): description(), image_path(), instance(), last_update(), params(), Convert a relative DB path to absolute using project root., # TODO: replace mock param with real get_params() once backend is stable, _resolve_path() (+7 more)

### Community 5 - "Config & Gazebo Process Control"
Cohesion: 0.05
Nodes (19): Config, CONFIG singleton, gazebo_is_running(), _is_gzserver_alive(), Call a ROS ServiceProxy in a daemon thread with a timeout.     Raises TimeoutErr, Block the worker thread until the user clicks Step.         Fires on_waiting_for, Grab one JPEG frame from the observer camera ROS topic.         Returns the last, Fire on_capture callback after each sensor capture.         Uses cached observer (+11 more)

### Community 6 - "Depth Camera Tests"
Cohesion: 0.09
Nodes (28): c3_view_angle_stability_test(), c5_working_range_test(), c6_small_displacement_sensitivity_test(), _camera_method_passed(), _choose_depth_topic(), _clean_mask(), _color_msg_to_bgr(), _depth_at_pixel_with_meta() (+20 more)

### Community 7 - "Stereo Camera Tests"
Cohesion: 0.1
Nodes (31): _bbox(), _camera_method_passed(), _clean_mask(), _count_pixels(), _crop_valid_ratio_bm(), _disparity_to_viz(), _ensure_render_display_env(), _expand_bbox() (+23 more)

### Community 8 - "UI Theme & Styling"
Cohesion: 0.11
Nodes (28): _to_absolute(), ADD(), CLEAR(), CLOSE(), CLOSE_BLK(), EDIT(), EXPORT(), FAIL() (+20 more)

### Community 9 - "Core, Detector & Sensor Abstraction"
Cohesion: 0.07
Nodes (20): ABC, Core, Read param values from SDF using the path/name defs stored in the sensor type DB, CUSTOM_DETECTORS dict, detect(), detect_sensor_type(), DETECTOR_REGISTRY, _load_detector_modules() (+12 more)

### Community 10 - "ROS/Catkin Build & Project Docs"
Cohesion: 0.07
Nodes (36): Monolithic PyQt5 process with Qt signals, Known issue: relative ./front/icon paths break if CWD not project root, Claude project context, Rationale: C9 FOV tolerance increased 2%->5% for Gazebo, SDF RFID summary (narrow+wide antenna), catkin_ws/src/CMakeLists.txt (symlink to ROS toplevel.cmake), gazebo_to_ros_tf (catkin), roscpp dep (gazebo_to_ros_tf) (+28 more)

### Community 11 - "Test Result Display Widget"
Cohesion: 0.15
Nodes (3): append_separator(), ColCapture, _on_capture_main()

### Community 12 - "RFID Tag Plugin (C++)"
Cohesion: 0.23
Nodes (13): expand_package_uri(), expand_tilde(), is_abs_path(), join_path(), ltrim_inplace(), read_map_robust(), readable_file(), resolve_custom_path() (+5 more)

### Community 13 - "RFID Antenna Plugin (C++)"
Cohesion: 0.22
Nodes (12): Antenna, expand_package_uri(), expand_tilde(), is_abs_path(), join_path(), readable_file(), resolve_custom_path(), RFID_ANTENNA (+4 more)

### Community 14 - "Setup & Launch Documentation"
Cohesion: 0.33
Nodes (6): Launch source-order (ROS -> catkin -> venv), Python 3.14 install, Rationale: must launch from project root due to relative resource paths, Rationale: ROS pkgs must precede venv because venv doesn't see rospy without source, ROS Noetic Installation, Troubleshoot: FileNotFoundError log_config.json

### Community 15 - "SQLite DB Portability Notes"
Cohesion: 0.67
Nodes (3): Known issue: sensor_storage.db not in .gitignore, First run creates empty sensor_storage.db, Rationale: delete DB when transferring between machines because of absolute paths

### Community 16 - "Test Registration Convention"
Cohesion: 0.67
Nodes (3): Test signature func(simulator,sensor,progress_cb)->dict, Add custom test guide, TESTS dict registration

### Community 17 - "Tactile SDF Rationale"
Cohesion: 0.67
Nodes (3): Rationale: sensor_size tag avoids conflict with box/size, Rationale: T2 threshold 10->150% because Gazebo physics on mm, SDF tactile summary (6 sensors, bumper plugin)

### Community 18 - "Tag Header Definitions"
Cohesion: 1.0
Nodes (0): 

### Community 19 - "Multi-plugin Detection Notes"
Cohesion: 1.0
Nodes (2): Counter-based multi-plugin detection, Namespace priority robotNamespace > namespace > cameraName

### Community 20 - "Capture Data Patterns Notes"
Cohesion: 1.0
Nodes (2): capture_data vs capture_persistent, OOM protection >1920x1080 no render, >2MB last frame only

### Community 21 - "Relative Path DB Convention"
Cohesion: 1.0
Nodes (2): Relative paths for portability in DB, Rationale: relative paths in DB enable multi-machine transfer

### Community 22 - "Antenna Header"
Cohesion: 1.0
Nodes (0): 

### Community 23 - "Spinner Animation Helper"
Cohesion: 1.0
Nodes (1): Returns a QMovie for the queued spinning animation.

### Community 24 - "Primary Topic Convention"
Cohesion: 1.0
Nodes (1): Primary topic — first in list, for backwards compat.

### Community 25 - "Roscore Connectivity Probe"
Cohesion: 1.0
Nodes (1): True while roscore is accepting connections on its port.         Checking the pr

### Community 26 - "ROS Node Init Probe"
Cohesion: 1.0
Nodes (1): True after rospy.init_node has succeeded.

### Community 27 - "Gazebo Liveness Probe"
Cohesion: 1.0
Nodes (1): True while Gazebo is running. Tries multiple detection strategies         becaus

### Community 28 - "UI Layout Helper"
Cohesion: 1.0
Nodes (1): Layout

### Community 29 - "Sensor Detector ABC"
Cohesion: 1.0
Nodes (1): SensorDetector ABC

### Community 30 - "rospkg Dependency"
Cohesion: 1.0
Nodes (1): rospkg

### Community 31 - "empy Dependency"
Cohesion: 1.0
Nodes (1): empy

### Community 32 - "PyYAML Dependency"
Cohesion: 1.0
Nodes (1): PyYAML

### Community 33 - "numpy Dependency"
Cohesion: 1.0
Nodes (1): numpy

### Community 34 - "PyQt5-sip Dependency"
Cohesion: 1.0
Nodes (1): PyQt5-sip

### Community 35 - "opencv-python Dependency"
Cohesion: 1.0
Nodes (1): opencv-python

### Community 36 - "defusedxml Dependency"
Cohesion: 1.0
Nodes (1): defusedxml

### Community 37 - "Roscore Port Conflict Note"
Cohesion: 1.0
Nodes (1): Troubleshoot: roscore port 11311 occupied

### Community 38 - "Headless Camera Note"
Cohesion: 1.0
Nodes (1): Troubleshoot: headless black camera screen

### Community 39 - "Empty Result Dict Bug"
Cohesion: 1.0
Nodes (1): Bug fix: empty result dict not sent to UI

### Community 40 - "Stereo Race Condition Bug"
Cohesion: 1.0
Nodes (1): Bug fix: race condition in scam_tests with lock

### Community 41 - "Cooperative Stop Refactor"
Cohesion: 1.0
Nodes (1): Bug fix: replaced PyThreadState_SetAsyncExc with cooperative stop

### Community 42 - "gzserver Pkill Hazard"
Cohesion: 1.0
Nodes (1): Known issue: pkill -f gzserver kills all gzserver processes

### Community 43 - "SDF Camera Catalog"
Cohesion: 1.0
Nodes (1): SDF camera summary (11 cameras)

## Knowledge Gaps
- **138 isolated node(s):** `Tag`, `Antenna`, `log_bridge.py  gazebo_simulator's log_config.json defines the "src" logger with`, `Add the Qt handler to a named logger if not already there.`, `Attach Qt handler to root and every logger that has propagate=False.` (+133 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Tag Header Definitions`** (2 nodes): `tags.h`, `tags()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Multi-plugin Detection Notes`** (2 nodes): `Counter-based multi-plugin detection`, `Namespace priority robotNamespace > namespace > cameraName`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Capture Data Patterns Notes`** (2 nodes): `capture_data vs capture_persistent`, `OOM protection >1920x1080 no render, >2MB last frame only`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Relative Path DB Convention`** (2 nodes): `Relative paths for portability in DB`, `Rationale: relative paths in DB enable multi-machine transfer`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Antenna Header`** (1 nodes): `antenna.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Spinner Animation Helper`** (1 nodes): `Returns a QMovie for the queued spinning animation.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Primary Topic Convention`** (1 nodes): `Primary topic — first in list, for backwards compat.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Roscore Connectivity Probe`** (1 nodes): `True while roscore is accepting connections on its port.         Checking the pr`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `ROS Node Init Probe`** (1 nodes): `True after rospy.init_node has succeeded.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Gazebo Liveness Probe`** (1 nodes): `True while Gazebo is running. Tries multiple detection strategies         becaus`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `UI Layout Helper`** (1 nodes): `Layout`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Sensor Detector ABC`** (1 nodes): `SensorDetector ABC`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `rospkg Dependency`** (1 nodes): `rospkg`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `empy Dependency`** (1 nodes): `empy`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `PyYAML Dependency`** (1 nodes): `PyYAML`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `numpy Dependency`** (1 nodes): `numpy`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `PyQt5-sip Dependency`** (1 nodes): `PyQt5-sip`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `opencv-python Dependency`** (1 nodes): `opencv-python`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `defusedxml Dependency`** (1 nodes): `defusedxml`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Roscore Port Conflict Note`** (1 nodes): `Troubleshoot: roscore port 11311 occupied`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Headless Camera Note`** (1 nodes): `Troubleshoot: headless black camera screen`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Empty Result Dict Bug`** (1 nodes): `Bug fix: empty result dict not sent to UI`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Stereo Race Condition Bug`** (1 nodes): `Bug fix: race condition in scam_tests with lock`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Cooperative Stop Refactor`** (1 nodes): `Bug fix: replaced PyThreadState_SetAsyncExc with cooperative stop`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `gzserver Pkill Hazard`** (1 nodes): `Known issue: pkill -f gzserver kills all gzserver processes`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `SDF Camera Catalog`** (1 nodes): `SDF camera summary (11 cameras)`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Layout` connect `Sensor & Type Dialogs` to `UI Theme & Styling`, `Test Queue & UI Items`, `Test Result Display Widget`, `Sensor Repository & Main Window`?**
  _High betweenness centrality (0.045) - this node is a cross-community bridge._
- **Why does `ColCapture` connect `Test Result Display Widget` to `Sensor & Type Dialogs`, `Test Queue & UI Items`, `Sensor Repository & Main Window`, `Config & Gazebo Process Control`, `UI Theme & Styling`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **Why does `Config` connect `Config & Gazebo Process Control` to `Stereo Camera Tests`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **Are the 72 inferred relationships involving `str` (e.g. with `._populate_params()` and `._load_params()`) actually correct?**
  _`str` has 72 INFERRED edges - model-reasoned connections that need verification._
- **Are the 57 inferred relationships involving `Layout` (e.g. with `AddSensorDialog` and `Convert absolute path to relative (from project root) for DB storage.`) actually correct?**
  _`Layout` has 57 INFERRED edges - model-reasoned connections that need verification._
- **Are the 15 inferred relationships involving `ColSensors` (e.g. with `Colors` and `Styles`) actually correct?**
  _`ColSensors` has 15 INFERRED edges - model-reasoned connections that need verification._
- **Are the 15 inferred relationships involving `ColTests` (e.g. with `Colors` and `Styles`) actually correct?**
  _`ColTests` has 15 INFERRED edges - model-reasoned connections that need verification._