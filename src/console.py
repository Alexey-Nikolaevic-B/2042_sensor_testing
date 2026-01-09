import sys
from os import system
from enum import Enum
from .sensors import make_sensor
from config import CONFIG

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
        system("clear")
        print("Типы датчиков: ")
        sensor_types = core.get_sensor_types()
        for i, type in enumerate(sensor_types, 1):
            print(f"[{i}] - {type}")
        
        category_choice = int(input("Выбор: ")) - 1
        if 0 <= category_choice < len(sensor_types):
            sensor_type = sensor_types[category_choice]
            print(f"\nДатчики типа '{sensor_type}':")
            sensors = core.get_sensors_name_by_type(sensor_type)
            for i, sensor_name in enumerate(sensors, 1):
                print(f"[{i}] - {sensor_name}")
            
            sensor_choice = int(input("Выбор: ")) - 1
            sensor_name = sensors[sensor_choice]
            if 0 <= sensor_choice < len(sensors):

                sensor = make_sensor(sensor_type, sensor_name, CONFIG)

                if not sensor:
                    system("clear")
                    print(f"Ошибка: датчик '{sensor_name}' не найден в типе '{sensor_type}'.")
                    return
                print('\n')

                sensor.print_params()

                print("Изменить параметры датчика?", "[1] - Да", "[2] - Нет", sep='\n')
                choice = int(input("Выбор: "))
                if choice == 1:
                    sensor.set_params()

                results = core.run_test(sensor)

                print()
                for result in results:
                    print('Test name:', result['test_name'])
                    print('Result:', result['result'], end='\n\n')
            else:
                system("clear")
                print("Ошибка: неверный выбор датчика.")
                return
        else:
            system("clear")
            print("Ошибка: неверный выбор типа датчика.")
            return
#
        
    elif choice == MenuChoice.EXIT.value:
        system("clear")
        sys.exit()
        
    else:
        system("clear")
        print("Ошибка: неверный выбор. Попробуйте снова.")
