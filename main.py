import sys

from os import system
from sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from test_manager import Test_manager

def console_menu(manager):
    print("\n1. Типы датчиков\n2. Список датчиков\n3. Запуск теста с захватом\n4. Выход")
    choice = input("Выберите опцию: ")
    if choice == "1":
        system("clear")
        print("Тип датчиков: ")
        for t in get_sensor_types(): print("-", t)
    elif choice == "2":
        system("clear")
        t = input("Типы датчика: ")
        for s in get_sensors_by_type(t): print("-", s["name"])
    elif choice == "3":
        system("clear")
        print("Типы датчиков: ")
        for t in get_sensor_types(): print("-", t)
        t = input("Типы: ")
        for s in get_sensors_by_type(t): print("-", s["name"])
        n = input("Имя: ")
        sensor = get_sensor(t, n)
        if not sensor:
            system("clear")
            print(f"Ошибка: датчик '{n}' не найден в типе '{t}'.")
            return
        system("clear")
        manager.launch()
        result = manager.new_sensor(t, sensor) #Todo: Придумать, как реализовать вывод
    elif choice == "4":
        manager.kill()
        sys.exit()
    else:
        system("clear")

if __name__ == '__main__':
    system("clear")
    manager = Test_manager()
    while True:
        console_menu(manager)
