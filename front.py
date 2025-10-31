from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal
import time
from functools import partial
from UI.components.ui_mainwindow import Ui_MainWindow
from UI.components.test_bar import TestBar
from UI.components.modal_info import ModalDialog
from UI.components.modal_result import TestResultDialog
from sensor_library import *
import os

# Поток для функции `wait_tests`
class WaitTestsThread(QThread):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window


    def run(self):
        datch_name = self.main_window.datch_name
        type_name = self.main_window.type_name
        while True:
            time.sleep(1)
            for test_entry in self.main_window.active_tests:
                try:
                    if test_entry["status"] == "In Progress":
                        test_bar = test_entry["test"]
                        if not hasattr(test_bar, "labelTitle") or not hasattr(test_bar.labelTitle, "text"):
                            continue

                        title = test_bar.labelTitle.text()
                        try:
                            test_name, status, result = get_test_results(datch_name, type_name, title)
                        except Exception as e:
                            continue

                        print("Перед обновлением test_entry:", test_entry)
                        test_entry["test_name"] = test_name
                        test_entry["status"] = status
                        test_entry["result"] = result
                        if status in ["Success", "Fail"]:
                            if hasattr(test_bar, "change_status"):
                                test_bar.change_status(status=status)
                            else:
                                print(f"Ошибка: test_bar не содержит метод change_status")
                except Exception as e:
                    print(f"Ошибка при обработке теста: {e}")

            try:
                all_ready = all(
                    test_entry["status"] != "In Progress"
                    for test_entry in self.main_window.active_tests
                )
                if all_ready:
                    print("Все тесты завершены!")
                    break
            except Exception as e:
                print(f"Ошибка при проверке завершения всех тестов: {e}")


class MainWindow(QtWidgets.QMainWindow):
    active_tests = []
    datch_name = ''
    type_name = ''

    def showModalDialog(self):
        type = self.type_name
        project_path = os.path.abspath(os.path.dirname(__file__))
        print(project_path)
        if type != "":
            img_url = project_path + "\img\\" + modal_data["type"][type]["img_url"]
            modal = ModalDialog(
                title=type,
                url=img_url,
                text=modal_data['type'][type]['article'],
                parent=self
            )

            modal.exec_()

    def showModalResultDialog(self, result, etalon):
        dialog = TestResultDialog(result, etalon, self)
        dialog.exec()

    def handleTestButtonClicked(self, test_name):
        # print(f"Кнопка нажата в тесте с названием: {test_name}")
        # print(self.active_tests)

        matching_test = next((test for test in self.active_tests if test['test_name'] == test_name), None)

        if matching_test:
            etalon = matching_test['etalon']
            result = matching_test['result']
            # print(f"Нашли тест: etalon={etalon}, result={result}")
            self.showModalResultDialog(result=result, etalon=etalon)
        else:
            pass
            # print(f"Тест с названием '{test_name}' не найден!")

    def start_test(self):
        self.update_tests()
        self.datch_name = self.ui.datch_menu.currentText()
        self.type_name = self.ui.type_menu.currentText()

        if self.datch_name != "Выбор датчика" and self.type_name != "Тип датчика":
            for test_name, test_values in get_tests_by_sensor(self.type_name, self.datch_name).items():
                etalon = test_values[0] if isinstance(test_values, tuple) else None
                result = test_values[1] if isinstance(test_values, tuple) else None

                test_bar = TestBar(
                    title=f"{test_name}",
                    status="In Progress"
                )

                self.ui.verticalLayout_2.addWidget(test_bar)
                self.active_tests.append({
                    "test": test_bar,
                    "test_name": test_name,
                    "status": "In Progress",
                    "etalon": etalon,
                    "result": result
                })

                test_bar.rightButton.clicked.connect(
                    partial(self.handleTestButtonClicked, test_name)
                )

            self.wait_thread = WaitTestsThread(self)
            self.wait_thread.start()

    def update_tests(self):
        while self.active_tests:
            test_entry = self.active_tests.pop()
            test_bar = test_entry["test"]
            self.ui.verticalLayout_2.removeWidget(test_bar)
            test_bar.hide()
            test_bar.deleteLater()

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.verticalLayout_2.removeWidget(self.ui.test_bar_6)
        self.ui.test_bar_6.setParent(None)
        self.ui.verticalLayout_2.removeWidget(self.ui.test_bar_7)
        self.ui.test_bar_7.setParent(None)

        self.ui.more_info.clicked.connect(self.showModalDialog)

        self.ui.start_test.clicked.connect(self.start_test)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
