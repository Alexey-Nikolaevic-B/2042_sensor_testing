from PyQt5 import QtWidgets, QtGui, QtCore


class TestResultDialog(QtWidgets.QDialog):
    """
    Модальное окно для вывода результатов теста.
    """

    def __init__(self, result, etalon, parent=None):
        super(TestResultDialog, self).__init__(parent)
        self.result = result
        self.etalon = etalon

        # Настройка пользовательского интерфейса
        self.setup_ui()

    def setup_ui(self):
        # Общие настройки окна
        self.setWindowTitle("Результат теста")
        self.setFixedSize(300, 200)  # Увеличиваем высоту окна для кнопки

        self.setStyleSheet(
            """
            QDialog {
                background-color: #E3E1E2;
                padding: 17px 12px 8px 12px;
            }
            QLabel#headingLabel {
                color: #545659;
                font-size: 20px;
                background-color: transparent;
            }
            QLabel#resultLabel, QLabel#etalonLabel {
                border: 2px solid #545659;
                background-color: #EEEEEE;
                padding: 5px;
                font-size: 22px;
                color: #545659;
                border-radius: 20px;
            }
            QPushButton#closeButton {
                background-color: #EEEEEE;
                color: #545659;
                border: 4px solid #545659;
                font-size: 16px;
                border-radius: 20px;
                font-weight: bold;
            }
            QPushButton#closeButton:hover {
                background-color: #CACACA;
                color: #545659;
            }
            QPushButton#closeButton:pressed {
                background-color: #B8B8B8;
            }
            """
        )

        # Главная вертикальная компоновка окна
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 20, 10, 0)

        # Надписи "Результат" и "Эталон" (отдельная строка)
        heading_layout = QtWidgets.QHBoxLayout()
        heading_layout.setSpacing(20)  # Расстояние между надписями
        heading_layout.setContentsMargins(0, 0, 0, 0)

        # Надпись "Результат"
        result_label = QtWidgets.QLabel("Результат", self)
        result_label.setObjectName("headingLabel")
        result_label.setFixedSize(100, 20)  # Фиксированный размер надписи
        result_label.setAlignment(QtCore.Qt.AlignCenter)
        heading_layout.addWidget(result_label)

        # Надпись "Эталон"
        etalon_label = QtWidgets.QLabel("Эталон", self)
        etalon_label.setObjectName("headingLabel")
        etalon_label.setFixedSize(100, 20)  # Фиксированный размер надписи
        etalon_label.setAlignment(QtCore.Qt.AlignCenter)
        heading_layout.addWidget(etalon_label)

        main_layout.addLayout(heading_layout)  # Добавляем строку с надписями в основной слой

        # Контейнеры для значений результата и эталона
        value_layout = QtWidgets.QHBoxLayout()
        value_layout.setSpacing(20)  # Расстояние между контейнерами
        value_layout.setContentsMargins(10, 0, 10, 0)

        # Контейнер с результатом
        result_container = self.create_value_container(self.result, is_result=True)
        value_layout.addWidget(result_container, alignment=QtCore.Qt.AlignCenter)

        # Контейнер с эталоном
        etalon_container = self.create_value_container(self.etalon, is_result=False)
        value_layout.addWidget(etalon_container, alignment=QtCore.Qt.AlignCenter)

        main_layout.addLayout(value_layout)  # Добавляем строку с контейнерами в основной слой

        # Кнопка "Закрыть"
        close_button = QtWidgets.QPushButton("Закрыть", self)
        close_button.setObjectName("closeButton")
        close_button.setFixedSize(160, 40)  # Размер кнопки
        close_button.clicked.connect(self.close)  # Событие для закрытия окна

        main_layout.addWidget(close_button, alignment=QtCore.Qt.AlignCenter)  # Добавляем кнопку в основной слой

    def create_value_container(self, value, is_result=False):
        # Контейнер для значения
        container = QtWidgets.QLabel(str(value), self)

        # Применение стилей
        if is_result:
            container.setObjectName("resultLabel")
            # Применяем логику цвета для результата
            try:
                numeric_value = float(value)
                numeric_etalon = float(self.etalon)
                if numeric_value < numeric_etalon:
                    color = "#BC0000"  # Красный, если результат меньше эталона
                elif numeric_value > numeric_etalon:
                    color = "#00B803"  # Зеленый, если результат больше эталона
                else:
                    color = "#545659"  # Нейтральный цвет
                container.setStyleSheet(f"color: {color};")
            except (ValueError, TypeError):
                # Если значения не поддаются сравнению, задаем нейтральный цвет
                container.setStyleSheet("color: #545659;")
        else:
            container.setObjectName("etalonLabel")

        # Размеры контейнера
        container.setFixedSize(80, 40)
        container.setAlignment(QtCore.Qt.AlignCenter)

        return container
