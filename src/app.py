import sys

from os import system

from .sensor_library import get_sensor_types, get_sensors_by_type, get_sensor
from .test_manager import TestManager
from .simulator.simulator_factory import SimulatorFactory

from config import CONFIG

class App:
    def __init__(self):
        self.simulator = SimulatorFactory.create_simulator('gazebo', CONFIG)
        self.test_manager = TestManager(self.simulator, CONFIG)

    def run(self):
        while True:
            self._display_menu()
            option = input("Выберите опцию: ")
            self._handle_option(option)

    def _display_menu(self):
        print("\n1. Типы датчиков")
        print("2. Список датчиков") 
        print("3. Запуск теста с захватом")
        print("4. Выход")

    def _handle_option(self, option):
        if option == "1":
            self._display_sensor_types()
        elif option == "2":
            self._display_sensors_by_type()
        elif option == "3":
            self._launch_sensor_test()
        elif option == "4":
            sys.exit()
        else:
            system("clear")
            print("Неверный выбор. Попробуйте снова.")

    def _display_sensor_types(self):
        system("clear")
        print("Типы датчиков: ")
        for sensor_type in get_sensor_types():
            print("-", sensor_type)

    def _display_sensors_by_type(self):
        system("clear")
        sensor_type = input("Тип датчика: ")
        print(f"Датчики типа '{sensor_type}':")
        for sensor in get_sensors_by_type(sensor_type):
            print("-", sensor["name"])

    def _launch_sensor_test(self):
        system("clear")
        print("Типы датчиков: ")
        for sensor_type in get_sensor_types():
            print("-", sensor_type)
        
        sensor_type = input("Тип: ")

        print(f"Датчики типа '{sensor_type}':")
        for sensor in get_sensors_by_type(sensor_type):
            print("-", sensor["name"])

        sensor_name = input("Имя: ")

        sensor = get_sensor(sensor_type, sensor_name)
        if not sensor:
            system("clear")
            print(f"Ошибка: датчик '{sensor_name}' не найден в типе '{sensor_type}'.")
            return
        system("clear")
        self.simulator.launch()
        result = self.test_manager.test_sensor(sensor_type, sensor)

    def exit(self):
        print('\nЗавершение процессов:')
        self.simulator.kill()
        sys.exit()