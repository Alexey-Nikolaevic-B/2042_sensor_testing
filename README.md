# 2042_sensor_testing

Тестовый стенд сенсоров на ROS Noetic + Gazebo.

Единый пайплайн для камер и RFID:
- подъем `roscore`/`roslaunch`,
- ожидание ROS master и sensor topics,
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
cd catkin_ws
catkin_make
source devel/setup.bash
```

## Запуск тестов (единый CLI)

Запуск из **любого каталога** — все пути вычисляются через `ROOT_PATH`.

```bash
source /opt/ros/noetic/setup.bash
source <repo>/catkin_ws/devel/setup.bash
```

### 1) Suite по всем камерам

```bash
python3 <repo>/__main__.py --sensor-type camera --all-sensors --suite
```

### 2) Suite для одной камеры

```bash
python3 <repo>/__main__.py --sensor-type camera --sensor-name d435_like --suite
```

### 3) Один тест одной камеры

```bash
python3 <repo>/__main__.py --sensor-type camera --sensor-name d435_like --test c5_working_range_test
```

### 4) RFID — точно так же

```bash
python3 <repo>/__main__.py --sensor-type rfid --sensor-name rfid --suite
```

### 5) Интерактивное консольное меню

```bash
python3 <repo>/__main__.py
```

> Без аргументов открывается старое интерактивное меню.

## CLI-Аргументы

| Аргумент | Описание |
|---|---|
| `--sensor-type` | Тип сенсора (`camera`, `rfid`) |
| `--sensor-name` / `--sensor` | Имя сенсора (например `d435_like`) |
| `--all-sensors` | Прогнать для **всех** сенсоров типа |
| `--test <name>` | Один тест (например `c1_size_order_test`) |
| `--suite` | Полный набор `*_test` методов |

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
