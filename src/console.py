import sys
from os import system

class ConsoleUi():
    def __init__(self, core):
        self.core = core

    def run(self):
        while True:
            self.menu()

    def menu(self):
        print("\n1. Типы датчиков\n2. Список датчиков\n3. Запуск теста с захватом\n4. Выход")
        choice = input("Выберите опцию: ")
        if choice == "1":
            system("clear")
            print("Тип датчиков: ")
            for t in self.core.get_sensor_types(): print("-", t)
        elif choice == "2":
            system("clear")
            t = input("Типы датчика: ")
            for s in self.core.get_sensors_by_type(t): print("-", s["name"])
        elif choice == "3":
            system("clear")
            print("Типы датчиков: ")
            for t in self.core.get_sensor_types(): print("-", t)
            t = input("Типы: ")
            for s in self.core.get_sensors_by_type(t): print("-", s["name"])
            n = input("Имя: ")
            sensor = self.core.get_sensor(t, n)
            if not sensor:
                system("clear")
                print(f"Ошибка: датчик '{n}' не найден в типе '{t}'.")
                return
            # system("clear")
            result = self.core.run_test(t, sensor)
        elif choice == "4":
            system("clear")
            sys.exit()
        else:
            system("clear")