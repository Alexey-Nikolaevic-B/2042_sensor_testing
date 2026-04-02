# Sensor Testing — Gazebo/ROS

Десктопное приложение (PyQt5) для автоматизированного тестирования датчиков в симуляторе Gazebo.
Поддерживаемые типы датчиков: mono/depth/stereo камеры, RFID, тактильные.

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
cd gazebo-sensors-validation-in-gazebo
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

### Добавление типа датчика

1. В левой колонкe нажмите кнопку **"+"**. Там две кнопки выбрать верхнюю.
2. Добваить новый тип датичка

### Добавление датчика

1. В левой колонкe нажмите кнопку **"+"**. Нижняя кнопка.
2. Выберите SDF-файл для датчика.
3. Тип датчика определится автоматически из SDF.
4. Подтвердите добавление.

### Запуск тестов

1. Выберите датчик (клик по нему).
2. В третей колонке появится список доступных тестов.
3. Нажмите кнопку запуска (**Play**) рядом с тестом или кнопку **"Run All"** для запуска всех.
4. Результаты отобразятся нв правой панели.



## Типичные проблы

### Приложение не запускается: `FileNotFoundError: log_config.json`

Приложение запущено не из корневой директории проекта.

```bash
cd /путь/к/gazebo-sensors-validation-in-gazebo
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
