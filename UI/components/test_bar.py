# components/test_bar.py
from PyQt5 import QtWidgets, QtCore
from UI.components.progress_bar_component import MyProgressBar

class TestBar(QtWidgets.QFrame):
    def change_status(self, status):
        try:
            # print("Изменяем статус виджета на:", status)

            if hasattr(self, 'labelStatus'):
                self.labelStatus.setText(status)

            if hasattr(self, 'circle'):
                if status == "Success":
                    self.circle.setStyleSheet("background: #25BB57;")  # Зеленый
                elif status == "Fail":
                    self.circle.setStyleSheet("background: #BB2525;")  # Красный

            if hasattr(self, 'progressBar') and self.progressBar:
                # print("Удаляем progressBar...")
                self.progressBar.setParent(None)  # Безопасное удаление

            # print("Макет и виджеты обновлены успешно.")

        except Exception as e:
            print(f"Ошибка в change_status: {e}")

    def __init__(self, parent=None, title="Тест", status="Success"):
        super(TestBar, self).__init__(parent)
        self.setupUi(title, status)

    def setupUi(self, title, status):
        self.setObjectName("TestBarFrame")
        self.setMinimumSize(QtCore.QSize(476, 118))
        self.setMaximumSize(QtCore.QSize(476, 118))
        self.setStyleSheet("""
            QFrame#TestBarFrame {
                background: #EEEEEE;
                border: 1px solid #545659;
                border-radius: 15px;
                padding: 8px 12px;
            }
            QLabel {
                color: #545659;
                font-size: 16px;
                background: #EEEEEE;
            }
            QPushButton {
                border: 1px solid #545659;
                background: #D9D9D9;
                font-size: 14px;
            }
            QPushButton:hover {
                background: #CACACA;
            }
            QPushButton:pressed {
                background: #B8B8B8;
            }  
        """)
        # Заголовок
        self.labelTitle = QtWidgets.QLabel(self)
        self.labelTitle.setGeometry(QtCore.QRect(20, 10, 200, 40))
        self.labelTitle.setText(title)
        # self.labelTitle.setAlignment(QtCore.Qt.AlignStart)
        self.labelTitle.setObjectName("labelTitle")


            # Левая кнопка
        self.leftButton = QtWidgets.QPushButton(self)
        self.leftButton.setGeometry(QtCore.QRect(20, 70, 201, 31))
        self.leftButton.setText("Подробнее")
        self.leftButton.setObjectName("leftButton")
        self.leftButton.setStyleSheet("""border-radius: 7px;""")
        # Правая кнопка
        self.rightButton = QtWidgets.QPushButton(self)
        self.rightButton.setGeometry(QtCore.QRect(260, 70, 201, 31))
        self.rightButton.setText("Результат")
        self.rightButton.setObjectName("rightButton")
        self.rightButton.setStyleSheet("""border-radius: 7px;""")

        #Прогресс-бар
        self.progressBar = MyProgressBar(self)
        self.progressBar.setGeometry(QtCore.QRect(20, 70, 441, 31))
        self.progressBar.setValue(0)

        # Область статуса
        self.frameStatus = QtWidgets.QFrame(self)
        self.frameStatus.setGeometry(QtCore.QRect(330, 10, 132, 42))
        self.frameStatus.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frameStatus.setObjectName("frameStatus")
        self.frameStatus.setStyleSheet("""
                    background-color: #FFFFFF;
                    border: 1px solid #545659;
                    border-radius: 10px;
                """)
        # Кружок статуса
        self.circle = QtWidgets.QFrame(self.frameStatus)
        self.circle.setGeometry(QtCore.QRect(10, 10, 22, 22))
        self.circle.setStyleSheet("border-radius: 20px; border: none;")
        if status == "Success":
            self.circle.setStyleSheet("background: #25BB57;")
        elif status == "In Progress":
            self.circle.setStyleSheet("background: #F0D000;")
        elif status == "Fail":
            self.circle.setStyleSheet("background: #BB2525;")
        self.circle.setObjectName("circle")

        # Текст статуса
        self.labelStatus = QtWidgets.QLabel(self.frameStatus)
        self.labelStatus.setGeometry(QtCore.QRect(40, 1, 80, 40))
        self.labelStatus.setText(status)
        self.labelStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.labelStatus.setObjectName("labelStatus")
        self.labelStatus.setStyleSheet("""border: none;""")


