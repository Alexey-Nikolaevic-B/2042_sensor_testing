import sys

from os import system

from sensor_library import get_sensor_types, get_sensors_by_type, add_sensor, get_sensor
from test_runner import run_test

def console_menu():
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
        # print("Типы датчиков: ") # Временная мера для удобства
        # for t in get_sensor_types(): print("-", t)
        # t = input("Типы: ")
        # for s in get_sensors_by_type(t): print("-", s["name"])
        # n = input("Имя: ")
        t = 'camera'
        n = 'model'
        sensor = get_sensor(t, n)
        if not sensor:
            system("clear")
            print(f"Ошибка: датчик '{n}' не найден в типе '{t}'.")
            return
        system("clear")
        result = run_test(t, sensor) #Todo: Придумать, как реализовать вывод
    elif choice == "4":
        system("clear")
        sys.exit()
    else:
        system("clear")

if __name__ == '__main__':
    system("clear")
    while True:
        console_menu()
