import sys
from os import system
from enum import Enum
from typing import Optional
import ast

from config import CONFIG
from .sensors import make_sensor, Sensor
from .core import Core


class MenuChoice(Enum):
    SENSOR_TYPES = "1"
    SENSOR_LIST = "2"
    RUN_TEST = "3"
    CAPURE_DATA = '4'
    EXIT = "5"

def run(core: None) -> None:
    core.prepare_simulator()

    while True:
        menu(core)

def menu(core: Core) -> None:
    print("\n[1] Типы датчиков")
    print("[2] Список датчиков") 
    print("[3] Запуск теста с захватом")
    print("[4] Получить данные с датчика")
    print("[5] Выход")
    choice = input("Выбор: ")

    if choice == MenuChoice.SENSOR_TYPES.value:
        system("clear")
        print("Типы датчиков: ")
        for _, sensor_type in enumerate(core.get_sensor_types(), 1):
            print(f"{sensor_type}")
            
    elif choice == MenuChoice.SENSOR_LIST.value:
        system("clear")
        print("Типы датчиков: ")

        sensor_types = core.get_sensor_types()
        for i, sensor_type in enumerate(sensor_types, 1):
            print(f"[{i}] - {sensor_type}")
        
        choice_num = int(input("Выбор: ")) - 1
        if 0 <= choice_num < len(sensor_types):
            sensor_type = sensor_types[choice_num]
            print(f"\nДатчики типа '{sensor_type}':")
            sensors = core.get_sensors_name_by_type(sensor_type)
            for i, sensor_name in enumerate(sensors, 1):
                print(f"{sensor_name}")
        else:
            system("clear")
            print("Ошибка: неверный выбор.")
            return
            
    elif choice == MenuChoice.RUN_TEST.value:
        sensor = choice_sensor(core, CONFIG)
        if sensor is None:
            return

        print('Параметры датчика:', sensor.get_params())
        print("Изменить параметры датчика?", "[1] - Да", "[2] - Нет", sep='\n')
        choice = int(input("Выбор: "))
        if choice == 1:
            print('Введите все параметры датчика, которые хотите изменить, и их новые значения в формате key=value. Пустая строка - завершить ввод')
            kwargs = read_kwargs()
            sensor.set_params(**kwargs)

        tests_functions = core.get_tests(sensor)
        print()
        for test_name, test_func in tests_functions.items():
            print(test_name, 'started!')
            result = test_func(core.simulator)
            print('Result:', result, end='\n\n')

        
    elif choice == MenuChoice.CAPURE_DATA.value:
        sensor = choice_sensor(core, CONFIG)
        if sensor is None:
            return

        print('Параметры датчика:', sensor.get_params())
        print("Изменить параметры датчика?", "[1] - Да", "[2] - Нет", sep='\n')
        choice = int(input("Выбор: "))
        if choice == 1:
            sensor.set_params()

        world_path = input("Введите путь до .world, из которого нужно считать данные")

        print("Введите все нужные kwargs для считывания данных в формате key=value. Пустая строка - завершить ввод")
        kwargs = read_kwargs()
        core.capture_data(sensor, world_path, **kwargs)

    elif choice == MenuChoice.EXIT.value:
        system("clear")
        sys.exit()
        
    else:
        system("clear")
        print("Ошибка: неверный выбор. Попробуйте снова.")


def read_kwargs() -> dict:
    kwargs = {}
    while True:
        line = input("> ").strip()
        if not line:
            break
        if "=" not in line:
            print("Нужно в формате: key=value")
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        try:
            kwargs[key] = ast.literal_eval(value)
        except Exception:
            kwargs[key] = value
    return kwargs


def choice_sensor(core: Core, CONFIG) -> Optional[Sensor]:
    """"
    Функция для выбора датчика
    Вызывается только из консольного меню
    """
    system("clear")
    print("Типы датчиков: ")

    sensor_types = core.get_sensor_types()
    for i, sensor_type in enumerate(sensor_types, 1):
        print(f"[{i}] - {sensor_type}")

    category_choice = int(input("Выбор: ")) - 1

    if not (0 <= category_choice < len(sensor_types)):
        system("clear")
        print("Ошибка: неверный выбор.")
        return

    sensor_type = sensor_types[category_choice]
    system("clear")
    print(f"Датчики типа '{sensor_type}':")
    sensors = core.get_sensors_name_by_type(sensor_type)
    for i, sensor_name in enumerate(sensors, 1):
        print(f"[{i}] - {sensor_name}")
    
    sensor_choice = int(input("Выбор: ")) - 1
    if not(0 <= sensor_choice < len(sensors)):
        system("clear")
        print("Ошибка: неверный выбор.")
        return
    
    sensor_name = sensors[sensor_choice]
    sensor = make_sensor(sensor_type, sensor_name, CONFIG)

    if not sensor:
        system("clear")
        print(f"Ошибка: датчик '{sensor_name}' не найден в типе '{sensor_type}'.")
        return
    print('\n')

    system("clear")

    return sensor
