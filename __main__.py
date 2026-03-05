import sys
from PyQt5.QtWidgets import QApplication

from sensor_repository import SensorRepository
from ui_main import Main_UI

if __name__ == "__main__":
    mainApp = QApplication(sys.argv)

    # Initialise the singleton repository once, before any UI is created.
    # Pass the path to your data source here.
    # When you switch from mock data to a real database, update only this line.
    SensorRepository(data_path="_mock.json")   # creates the singleton

    app = Main_UI()
    app.show()

    try:
        sys.exit(mainApp.exec_())
    except Exception:
        print("Exiting")