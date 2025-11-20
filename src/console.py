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
    print("\n[1] Типы датчиков")
    print("[2] Список датчиков") 
    print("[3] Запуск теста с захватом")
    print("[4] Получить данные с датчика")
    print("[5] Выход")
    choice = input("Выбор: ")

    if choice == MenuChoice.SENSOR_TYPES.value:
        system("clear")
        print("Типы датчиков: ")
        for i, sensor_category in enumerate(core.get_sensor_categories(), 1):
            print(f"- {sensor_category}")
            
    elif choice == MenuChoice.SENSOR_LIST.value:
        system("clear")
        print("Типы датчиков: ")
        sensor_categories = list(core.get_sensor_categories())
        for i, category in enumerate(sensor_categories, 1):
            print(f"[{i}] {category}")
        
        choice_num = int(input("Выбор: ")) - 1
        if 0 <= choice_num < len(sensor_categories):
            sensor_category = sensor_categories[choice_num]
            print(f"\nДатчики типа '{sensor_category}':")
            sensors = core.get_sensors_by_model(sensor_category)
            for i, sensor_model in enumerate(sensors, 1):
                print(f"- {sensor_model['name']}")
        else:
            system("clear")
            print("Ошибка: неверный выбор.")
            return
            
    elif choice == MenuChoice.RUN_TEST.value:
        system("clear")
        print("Типы датчиков: ")
        sensor_categories = list(core.get_sensor_categories())
        for i, category in enumerate(sensor_categories, 1):
            print(f"[{i}] {category}")
        
        category_choice = int(input("Выбор: ")) - 1
        if 0 <= category_choice < len(sensor_categories):
            sensor_category = sensor_categories[category_choice]
            print(f"\nДатчики типа '{sensor_category}':")
            sensors = core.get_sensors_by_model(sensor_category)
            for i, sensor_model in enumerate(sensors, 1):
                print(f"[{i}] {sensor_model['name']}")
            
            sensor_choice = int(input("Выбор: ")) - 1
            if 0 <= sensor_choice < len(sensors):
                sensor_model = sensors[sensor_choice]["name"]
                sensor = core.get_sensor(sensor_category, sensor_model)
                if not sensor:
                    system("clear")
                    print(f"Ошибка: датчик '{sensor_model}' не найден в типе '{sensor_category}'.")
                    return
                print('\n')
                result = core.run_test(sensor_category, sensor)

                test_name = result[0]['test_name']
                result_score = result[0]['result']

                print(f'\n{test_name}')
                print(result_score)
            else:
                system("clear")
                print("Ошибка: неверный выбор датчика.")
                return
        else:
            system("clear")
            print("Ошибка: неверный выбор типа датчика.")
            return

    elif choice == MenuChoice.CAPURE_DATA.value:
        system("clear")
        print("Типы датчиков: ")
        sensor_categories = list(core.get_sensor_categories())
        for i, category in enumerate(sensor_categories, 1):
            print(f"[{i}] {category}")
        
        category_choice = int(input("Выбор: ")) - 1

        if 0 <= category_choice < len(sensor_categories):
            sensor_category = sensor_categories[category_choice]
            print(f"\nДатчики типа '{sensor_category}':")
            sensors = core.get_sensors_by_model(sensor_category)
            for i, sensor_model in enumerate(sensors, 1):
                print(f"[{i}] {sensor_model['name']}")
            
            sensor_choice = int(input("Выбор: ")) - 1
            if 0 <= sensor_choice < len(sensors):
                sensor_model = sensors[sensor_choice]["name"]
                sensor = core.get_sensor(sensor_category, sensor_model)
                if not sensor:
                    system("clear")
                    print(f"Ошибка: датчик '{sensor_model}' не найден в типе '{sensor_category}'.")
                    return

                world_path = 'resources/worlds/mono_camera/example.world'
                camera_model_path = 'resources/sensors/camera/mono_camera.sdf'
                print('\n')
                core.capture_data(camera_model_path, world_path)
            else:
                system("clear")
                print("Ошибка: неверный выбор датчика.")
                return
        else:
            system("clear")
            print("Ошибка: неверный выбор типа датчика.")
            return

        
    elif choice == MenuChoice.EXIT.value:
        system("clear")
        sys.exit()
        
    else:
        system("clear")
        print("Ошибка: неверный выбор. Попробуйте снова.")