import sys
from os import system
from enum import Enum

class MenuChoice(Enum):
    SENSOR_TYPES = "1"
    SENSOR_LIST = "2"
    RUN_TEST = "3"
    CAPURE_DATA = '4'
    EXIT = "5"

def run(core):
    core.prepare_simulator()

    while True:
        menu(core)

def menu(core):
    print("\n1. Типы датчиков\n2. Список датчиков\n3. Запуск теста с захватом\n4. Получить данные с датчика\n5. Выход")
    choice = input("Выберите опцию: ")

    if choice == MenuChoice.SENSOR_TYPES.value:
        system("clear")
        print("Типы датчиков: ")
        for sensor_category in core.get_sensor_categories(): 
            print("-", sensor_category)
            
    elif choice == MenuChoice.SENSOR_LIST.value:
        system("clear")
        sensor_category = input("Типы датчика: ")
        for sensor_model in core.get_sensors_by_model(sensor_category): 
            print("-", sensor_model["name"])
            
    elif choice == MenuChoice.RUN_TEST.value:
        system("clear")
        print("Типы датчиков: ")
        for sensor_category in core.get_sensor_categories(): 
            print("-", sensor_category)
        sensor_category = input("Типы: ")
        for sensor_model in core.get_sensors_by_model(sensor_category): 
            print("-", sensor_model["name"])
        sensor_model = input("Имя: ")
        sensor = core.get_sensor(sensor_category, sensor_model)
        if not sensor:
            system("clear")
            print(f"Ошибка: датчик '{sensor_model}' не найден в типе '{sensor_category}'.")
            return
        result = core.run_test(sensor_category, sensor)

    elif choice == MenuChoice.CAPURE_DATA.value:
        system("clear")
        print("Типы датчиков: ")
        for sensor_category in core.get_sensor_categories(): 
            print("-", sensor_category)
        sensor_category = input("Типы: ")
        for sensor_model in core.get_sensors_by_model(sensor_category): 
            print("-", sensor_model["name"])
        sensor_model = input("Имя: ")
        sensor = core.get_sensor(sensor_category, sensor_model)
        if not sensor:
            system("clear")
            print(f"Ошибка: датчик '{sensor_model}' не найден в типе '{sensor_category}'.")
            return
        core.capture_data(sensor_category, sensor)
        
    elif choice == MenuChoice.EXIT.value:
        system("clear")
        sys.exit()
        
    else:
        system("clear")