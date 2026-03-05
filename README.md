# 2042_sensor_testing

Тестовый стенд сенсоров на ROS Noetic + Gazebo.

Текущий фокус: стабильный пайплайн камер через единый runner:
- подъем `roscore`/`roslaunch`,
- ожидание ROS master и image topics,
- запуск `*_test`,
- сбор `report.json` + диагностик,
- корректное завершение только своих процессов.

## Требования

- Ubuntu + ROS Noetic + Gazebo (classic, `gazebo_ros`).
- Python 3.
- Пакеты Python:
  - `PyYAML`
  - `numpy`
  - `opencv-python`
  - `rospy`, `sensor_msgs`, `gazebo_msgs` (из ROS окружения)

## Подготовка catkin_ws

```bash
cd /Users/arturkuanyshev/Projects/2042_sensor_testing/catkin_ws
catkin_make
source devel/setup.bash
```

## Быстрый запуск камерных тестов

Запуск возможен из любого `cwd`, если путь к `test_runner.py` указан абсолютный.

### 1) Полный suite по всем камерам

```bash
source /opt/ros/noetic/setup.bash
source /Users/arturkuanyshev/Projects/2042_sensor_testing/catkin_ws/devel/setup.bash
python3 /Users/arturkuanyshev/Projects/2042_sensor_testing/test_runner.py --sensor-type camera --all-sensors --suite
```

### 2) Suite для одной камеры

```bash
source /opt/ros/noetic/setup.bash
source /Users/arturkuanyshev/Projects/2042_sensor_testing/catkin_ws/devel/setup.bash
python3 /Users/arturkuanyshev/Projects/2042_sensor_testing/test_runner.py --sensor-type camera --sensor d435_like --suite
```

### 3) Один тест для одной камеры

```bash
source /opt/ros/noetic/setup.bash
source /Users/arturkuanyshev/Projects/2042_sensor_testing/catkin_ws/devel/setup.bash
python3 /Users/arturkuanyshev/Projects/2042_sensor_testing/test_runner.py --sensor-type camera --sensor d435_like --test c5_working_range_test
```

## Интерактивный CLI

Старый консольный интерфейс:

```bash
source /opt/ros/noetic/setup.bash
source /Users/arturkuanyshev/Projects/2042_sensor_testing/catkin_ws/devel/setup.bash
python3 /Users/arturkuanyshev/Projects/2042_sensor_testing/__main__.py
```

## Отчеты и диагностика

- По сенсору: `results/<sensor_name>/report.json`
- По запуску всех сенсоров типа: `results/<sensor_type>_suite/report.json`
- Логи ROS/Gazebo: `ros_log/`
- Артефакты падений: debug PNG рядом с `report.json`

При падении теста в `error_details` пишутся:
- `world_path_abs`, `generated_world_path_abs`, `cwd`
- `launch_stderr_tail`/`launch_stdout_tail`
- `missing_model_uris`
- `expected_topics`, `resolved_topics`
- `msg_counters`, `msg_hz`
- `rostopic_list`
