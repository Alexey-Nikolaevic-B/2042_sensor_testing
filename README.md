# Sensor Testing — Gazebo/ROS

Десктопное приложение (PyQt5) для автоматизированного тестирования датчиков в симуляторе Gazebo.
Поддерживаемые типы датчиков: mono/depth/stereo камеры, RFID, тактильные.

---

## Системные требования

| Компонент | Версия | Примечание |
|-----------|--------|------------|
| ОС | Ubuntu 20.04 LTS | Единственная официально поддерживаемая |
| ROS | Noetic | Последняя версия для Ubuntu 20.04 |
| Gazebo | 11 | Устанавливается вместе с ROS Noetic |
| Python | 3.14+ | Должен быть установлен отдельно |
| Qt | PyQt5 | Устанавливается через pip |

---

## Установка

### 1. Установите ROS Noetic

Следуйте официальной инструкции: http://wiki.ros.org/noetic/Installation/Ubuntu

После установки убедитесь, что ROS доступен:

```bash
source /opt/ros/noetic/setup.bash
roscore  # Должен запуститься без ошибок, остановите через Ctrl+C
```

### 2. Установите Python 3.14

```bash
sudo apt update
sudo apt install python3.14 python3.14-venv python3.14-dev
```

Проверка:

```bash
python3.14 --version
```

### 3. Клонируйте репозиторий

```bash
git clone <URL_репозитория>
cd 2042_sensor_testing
```

### 4. Запустите скрипт установки

```bash
chmod +x setup.sh
./setup.sh
```

Скрипт автоматически:
- Проверит наличие ROS Noetic и Python 3.14
- Установит ROS-зависимости (MAVROS, Gazebo-плагины)
- Создаст виртуальное окружение (`venv/`)
- Установит Python-зависимости из `requirements.txt`
- Соберёт catkin workspace (`catkin_ws/`)

### 5. Запуск приложения

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
source venv/bin/activate
python3 __main__.py
```

**Важно:**
- Все три `source` нужны **в этом порядке**: сначала ROS, потом catkin, потом venv.
- Приложение **должно запускаться из корневой директории проекта** — пути к ресурсам привязаны к ней.
- Чтобы не вводить `source` каждый раз, добавьте первые две строки в `~/.bashrc`.

---

## Первый запуск

При первом запуске база данных (`sensor_storage.db`) создаётся автоматически. Она будет пустой — без датчиков.

### Добавление датчика

1. В левой колонке (Col 1) нажмите кнопку **"+"**.
2. В диалоге выберите SDF-файл датчика из папки `assets/sensors/`:
   - `camera_example/model.sdf` — mono камера
   - `depth_example/model.sdf` — depth камера
   - `rfid_example/model.sdf` — RFID датчик
   - `tactile_example/model.sdf` — тактильный датчик
3. Тип датчика определится автоматически из SDF.
4. Подтвердите добавление.

### Запуск тестов

1. Выберите датчик в Col 1 (клик по нему).
2. В Col 3 появится список доступных тестов.
3. Нажмите кнопку запуска (**Play**) рядом с тестом или кнопку **"Run All"** для запуска всех.
4. Результаты отобразятся в Col 4 (правая панель).

---

## Структура проекта

```
.
├── __main__.py              # Точка входа
├── config.py                # Конфигурация (читает config.yaml)
├── config.yaml              # Настройки путей, таймаутов
├── requirements.txt         # Python-зависимости
├── setup.sh                 # Скрипт установки
├── log_config.json          # Настройки логирования
│
├── front/                   # Интерфейс (PyQt5)
│   ├── main.py              # Главное окно (4 колонки)
│   ├── widget_col_1.py      # Список датчиков
│   ├── widget_col_2.py      # Детали датчика
│   ├── widget_col_3.py      # Список тестов
│   ├── widget_col_4.py      # Результаты, логи, изображения
│   ├── logic_queue_manager.py   # Очередь тестов
│   ├── logic_test_runner.py     # Выполнение тестов
│   ├── logic_sensor_repository.py # Кэш датчиков + SQLite
│   ├── dialog_add_sensor.py     # Диалог добавления датчика
│   ├── dialog_add_sensor_type.py # Диалог типа датчика
│   ├── dialog_edit_tests.py     # Редактирование тестов
│   ├── _theme.py            # Цвета и стили
│   ├── icon/                # Иконки
│   └── qt/                  # Qt-ресурсы
│
├── src/                     # Бэкенд
│   ├── core.py              # Ядро: связь с Simulator + реестр тестов
│   ├── sensor.py            # Модель датчика, захват данных
│   ├── sensor_storage.py    # SQLite: датчики, результаты, типы
│   ├── gazebo_simulator.py  # Управление ROS/Gazebo
│   ├── detector.py          # Автоопределение типа датчика из SDF
│   └── tests/               # Тестовые функции
│       ├── _common.py       # Реестр TESTS, утилиты, Worlds
│       ├── generic_tests.py # Базовый тест (sensor_capture_basic)
│       ├── mcam_tests.py    # Тесты mono камеры (C1-C11)
│       ├── dcam_tests.py    # Тесты depth камеры (C3, C5, C6)
│       ├── scam_tests.py    # Тесты stereo камеры (S1, S2)
│       ├── rfid_tests.py    # Тесты RFID
│       └── tactile_tests.py # Тесты тактильных датчиков (T1-T4)
│
├── assets/
│   ├── sensors/             # Примеры SDF-файлов датчиков
│   ├── worlds/              # World-файлы для Gazebo (сцены тестов)
│   └── observer_camera.sdf  # Камера-наблюдатель
│
├── catkin_ws/               # ROS workspace
│   ├── scenario_test_pkg/   # Пакет запуска сценариев
│   └── RFID_Sensor_Plugin_gazebo/ # RFID-плагин для Gazebo
│
└── sensors/                 # Директория для пользовательских датчиков
```

---

## Решение типичных проблем

### Приложение не запускается: `FileNotFoundError: log_config.json`

Приложение запущено не из корневой директории проекта.

```bash
cd /путь/к/2042_sensor_testing
source venv/bin/activate
python3 __main__.py
```

### Ошибка: `No such file or directory: .../model.sdf`

База данных содержит ссылки на датчики с путями от предыдущей машины. Удалите базу и пересоздайте:

```bash
rm sensor_storage.db
python3 __main__.py
# После запуска заново добавьте датчики через интерфейс
```

### Ошибка: `roscore` не запускается / порт 11311 занят

Другой экземпляр ROS уже работает.

```bash
# Найти и завершить процессы
killall -9 roscore rosmaster gzserver gzclient
# Подождать 2–3 секунды и запустить приложение заново
```

### Тест зависает, Gazebo не отвечает

Используйте кнопку **Stop** в интерфейсе. Если не помогает — **Force Kill** (принудительная остановка). Это убьёт процессы Gazebo и вернёт управление.

Если интерфейс полностью завис:

```bash
# В другом терминале
killall -9 gzserver gzclient
# Перезапустите приложение
```

### Тесты проходят с `passed: False`, но ошибки нет

Откройте Col 4 (правая панель) — в блоке **metrics** будет описание причины. Типичные причины:
- **`scene_open_failed`** — Gazebo не смог открыть сцену. Проверьте, что world-файл существует в `assets/worlds/`.
- **`model_not_spawned`** — модель не появилась в сцене. Перезапустите тест.
- **`image_receive_failed`** — камера не отправила кадр. Проверьте топик в SDF.
- **`resolution_mismatch`** — фактическое разрешение не совпадает с SDF. Проверьте параметры камеры.

### Ошибка: `No module named 'rospy'`

Виртуальное окружение не видит ROS-пакеты. Причина: не выполнен `source` перед запуском.

```bash
# Правильный порядок — все три source, затем запуск:
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
source venv/bin/activate
python3 __main__.py
```

Если ошибка сохраняется — проверьте, что catkin workspace собран:

```bash
ls catkin_ws/devel/setup.bash   # Файл должен существовать
# Если нет — пересоберите:
cd catkin_ws && catkin_make && cd ..
```

### Ошибка: `catkin_make failed`

Чаще всего причина — не установлены MAVROS-пакеты (нужны для RFID-плагина):

```bash
# Установите зависимости вручную
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-libmavconn ros-noetic-mavlink
sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Очистите и пересоберите
cd catkin_ws
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make
cd ..
```

Если ошибка `Could not find a package configuration file provided by "mavros_msgs"` — значит пакет `ros-noetic-mavros-msgs` не установился. Проверьте, что ROS-репозиторий добавлен в apt sources.

### Ошибка: `PyQt5` не устанавливается

```bash
sudo apt install python3.14-dev libgl1-mesa-dev
source venv/bin/activate
pip install PyQt5
```

### Чёрный экран / нет изображения с камеры в тестах

Gazebo запущен без графического окружения (headless). Тесты камер требуют рендеринг. Убедитесь:

```bash
# Проверьте что DISPLAY установлен
echo $DISPLAY   # Должен быть :0 или :1

# Если пусто — запустите с виртуальным дисплеем:
export DISPLAY=:0
python3 __main__.py
```

### После обновления кода тесты ведут себя странно

Удалите старую БД и `__pycache__`:

```bash
rm sensor_storage.db
find . -type d -name __pycache__ -exec rm -rf {} +
python3 __main__.py
```

---

## Конфигурация

Файл `config.yaml` в корне проекта. Основные параметры:

| Параметр | Значение по умолчанию | Описание |
|----------|----------------------|----------|
| `CATKIN_SETUP_DIR` | `catkin_ws/devel/setup.bash` | Путь к setup.bash catkin workspace |
| `SENSOR_PKG` | `scenario_test_pkg` | ROS-пакет для запуска сценариев |
| `LAUNCH_FILE` | `scenario.launch` | Файл запуска Gazebo |
| `WORLDS_PATH` | `resources/worlds/` | Директория с world-файлами |
| `SENSORS_PATH` | `resources/sensors/` | Директория с SDF датчиков |
| `MESSAGE_TIMEOUT` | `10` | Таймаут ожидания ROS-сообщения (секунды) |
| `SAVE_SENSOR_DATA` | `true` | Сохранять данные датчиков |
| `SAVE_DEBUG_DATA` | `true` | Сохранять отладочные данные |

---

## Добавление собственных тестов

1. Создайте функцию в соответствующем файле (`src/tests/mcam_tests.py`, и т.д.):

```python
def my_custom_test(simulator, sensor, progress_cb=None) -> dict:
    # Ваша логика тестирования
    result = {"passed": True, "metrics": {...}}
    return result
```

2. Зарегистрируйте тест в `src/tests/_common.py`:

```python
TESTS["my_custom_test"] = my_custom_test
```

3. Привяжите тест к типу датчика через интерфейс: Col 1 → Редактировать тип → Добавить тест.
