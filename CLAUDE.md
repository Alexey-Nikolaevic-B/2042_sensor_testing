# 2042 Sensor Testing — Рабочая директория

## Обзор проекта
Платформа для тестирования датчиков в симуляции Gazebo с UI на PyQt5.
Пользователь через UI добавляет датчик, загружает SDF-плагин, задаёт параметры датчика и выбирает тесты.

## Архитектура
- **front/** — PyQt5 GUI (виджеты колонок, диалоги добавления датчиков, редактирования тестов)
- **src/core.py** — ядро приложения
- **src/gazebo_simulator.py** — обёртка над ROS/Gazebo (launch, set_pose, open_scene, capture)
- **src/tests/tests.py** — ВСЕ тесты (RFID, камеры mono/depth/stereo, tactile)
- **src/sensors/** — детектор датчиков, базовый класс Sensor
- **src/database/** — SQLite хранилище датчиков и типов
- **resources/** — SDF-файлы датчиков, .world-файлы сцен
- **assets/** — конфигурации датчиков и мир-файлы для тестов камер (c1-c11, depth, rfid, tactile)
- **catkin_ws/** — ROS catkin workspace (RFID плагины, scenario launch)

## Тесты в tests.py (src/tests/tests.py)
Все тесты — функции вида `def test_name(simulator, sensor, progress_cb=None) -> dict`.
Регистрируются в словаре `TESTS` в конце файла.

### Типы камерных тестов
- **Mono** (C1-C11): используют `_mono_build_ctx(sensor)` → SimpleNamespace ctx
  - C1 size_order, C2 resolution, C4 geometries, C7 occlusion, C9 fov, C10 clipping, C11 fps
- **Depth**: класс `_DepthProfileTestContext` (строка ~2613)
  - depth_perception, c3_view_angle, c5_working_range, c6_small_displacement
- **Stereo**: класс `_StereoProfileTestContext` (строка ~4089)
  - stereo_topics_presence, stereo_disparity, stereo_occlusion, s1_accuracy, s2_texture

### Паттерн вызова
```python
# Mono — через _run_camera_function_test(_mono_build_ctx, _mono_c1_test, ...)
# Depth/Stereo — через _run_camera_context_test(ContextClass, "method_name", ...)
```

### Регистрация нового теста
1. Написать функцию `def my_test(simulator, sensor, progress_cb=None) -> dict`
2. Добавить в словарь `TESTS` в конце файла
3. Добавить world-файл в `assets/worlds/` и запись в `Worlds` enum если нужно
4. В UI назначить тест типу датчика

## Simulator API (src/gazebo_simulator.py)
- `simulator.open_scene(world_path, sdf_path) -> bool`
- `simulator.wait_for_model_spawn(model_name, timeout) -> bool`
- `simulator.set_pose(model, x, y, z, quaternion=None, ...)`
- `simulator.kill_gazebo()` / `simulator.kill()`
- `simulator.capture_observer_frame() -> bytes | None`

## Sensor API
- `sensor.sensor_type`, `sensor.sensor_name`, `sensor.sdf_path`, `sensor.topic`
- `sensor.params` — dict параметров
- `sensor.capture_data(msg_type, window, timeout, simulator) -> list[msg]`
- `sensor.capture_frames(msg_type, window, simulator) -> dict` (RFID)

## Миграция из старого репо (2042_sensor_testing)
Старый репо: тесты были в `src/sensors/` как классы (MonoProfileBase, DepthProfileBase, StereoProfileBase).
Новый репо: все тесты — плоские функции в `src/tests/tests.py`.

### Статус переноса
- [x] Mono тесты (C1, C2, C4, C7, C9, C10, C11)
- [x] Depth тесты (depth_perception, c3, c5, c6)
- [x] Stereo тесты (topics_presence, disparity, occlusion, s1, s2)
- [x] RFID тесты
- [x] Tactile тесты
- [x] SDF parsing (_camera_load_sensor_profile)
- [x] World файлы (assets/worlds/)

## Важные пути
- ROOT_PATH: определяется в config.yaml
- World файлы: `{ROOT_PATH}/assets/worlds/`
- SDF датчиков: `resources/sensors/camera/`
- Captured data: `captured_data/`
- Results: `results/{sensor_name}/`
