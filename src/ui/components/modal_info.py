import os
from PyQt5 import QtWidgets, QtGui, QtCore


class RoundedImageLabel(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.original_pixmap = None  # Оригинальный pixmap для отрисовки

    def setPixmap(self, pixmap):
        if not pixmap or pixmap.isNull():
            print("Ошибка: Передан пустой pixmap")  # Лог для отладки
            return
        self.original_pixmap = pixmap
        self.repaint()  # Сразу вызываем перерисовку виджета

    def paintEvent(self, event):
        border_width = 2  # Толщина рамки (должна совпадать с CSS)

        # Создаём Painter для отрисовки
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)

        # Создаём скруглённый путь для внутренних границ
        path = QtGui.QPainterPath()
        path.addRoundedRect(
            border_width,  # Учитываем отступ сверху (рамка)
            border_width,  # Учитываем отступ слева (рамка)
            self.width() - 2 * border_width,  # Ширина с вычетом рамки
            self.height() - 2 * border_width,  # Высота с вычетом рамки
            10,  # Радиус скругления (остается таким же)
            10  # Радиус скругления (остается таким же)
        )

        # Устанавливаем область отрисовки
        painter.setClipPath(path)

        # Если изображение задано, рисуем его
        if self.original_pixmap:
            scaled_pixmap = self.original_pixmap.scaled(
                self.width() - 2 * border_width,  # Масштабируем ширину
                self.height() - 2 * border_width,  # Масштабируем высоту
                QtCore.Qt.IgnoreAspectRatio,  # Игнорируем пропорции
                QtCore.Qt.SmoothTransformation  # Используем плавное масштабирование
            )
            # Отрисовываем изображение с учётом рамки
            painter.drawPixmap(border_width, border_width, scaled_pixmap)
        else:
            # Если изображение отсутствует, заполняем белым цветом
            painter.fillRect(
                border_width,
                border_width,
                self.width() - 2 * border_width,
                self.height() - 2 * border_width,
                QtCore.Qt.white
            )
            painter.drawText(
                self.rect(),
                QtCore.Qt.AlignCenter,
                "Нет изображения"
            )

        painter.end()


class ModalDialog(QtWidgets.QDialog):
    """
    Компонент модального окна.
    Принимает title, url и text, отображает данные через интерфейс.
    """

    def __init__(self, title="Заголовок", url="", text="", parent=None):
        super(ModalDialog, self).__init__(parent)
        self.title = title

        # Корневая папка проекта
        project_root = os.path.dirname(os.path.abspath(__file__))  # Папка текущего файла
        # project_root = os.path.abspath(os.path.join(project_root, ".."))  # Переход в корневую папку проекта

        # Путь к изображению
        self.url = url  # Построение пути (корень + относительный путь)
        self.text = text

        self.setupUi()

    def setupUi(self):
        # Устанавливаем общие параметры для диалога
        self.setWindowTitle(self.title)
        self.setFixedSize(750, 500)

        self.setStyleSheet(
            """
            QDialog {
                background-color: #E3E1E2;
                border: 1px solid #545659;
                padding: 17px 10px 10px 10px;
            }
            QLabel#titleLabel {
                color: #545659;
                font-size: 20px;
                font-weight: normal;
            }
            QLabel#imageLabel {
                border: 2px solid #090909;
                border-radius: 10px;
            }
            QLabel#text_label {
                color: #494A4D;
                font-size: 15px;
            }
            QPushButton#closeButton {
                border: 3px solid #545659;
                border-radius: 20px;
                width: 300px;
                height: 50px;
                font-size: 20px;
                color: #494A4D;
            }
            QPushButton#closeButton:hover {
                background: #D8D8D8;
            }
            QPushButton#closeButton:pressed {
                background: #C4C4C4;
            }
            """
        )

        # Настраиваем layout
        layout = QtWidgets.QVBoxLayout(self)

        # Заголовок
        self.title_label = QtWidgets.QLabel(self.title, self)
        self.title_label.setObjectName("titleLabel")
        self.title_label.setAlignment(QtCore.Qt.AlignLeft)
        layout.addWidget(self.title_label)

        layout.addSpacing(10)

        # Создаём RoundedImageLabel для изображения
        self.image_label = RoundedImageLabel(self)
        self.image_label.setObjectName("imageLabel")
        self.image_label.setFixedSize(350, 250)  # Устанавливаем размеры контейнера
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)

        # Загрузка изображения
        pixmap = QtGui.QPixmap(self.url)
        if not pixmap.isNull():  # Проверяем, что изображение доступно
            self.image_label.setPixmap(pixmap)
        else:
            print(f"Ошибка: Изображение не найдено по пути {self.url}")  # Лог ошибок
            self.image_label.setText("Изображение недоступно")
            self.image_label.setAlignment(QtCore.Qt.AlignCenter)

        # Добавляем виджет с изображением
        layout.addWidget(self.image_label, alignment=QtCore.Qt.AlignHCenter)

        layout.addSpacing(10)

        # Текст
        self.text_label = QtWidgets.QLabel(self.text, self)
        self.text_label.setWordWrap(True)
        self.text_label.setAlignment(QtCore.Qt.AlignLeft)
        self.text_label.setStyleSheet("font-size: 16px; color: #494A4D;")
        layout.addWidget(self.text_label)

        layout.addSpacing(10)

        # Кнопка закрытия
        self.close_button = QtWidgets.QPushButton("Закрыть", self)
        self.close_button.setObjectName("closeButton")
        self.close_button.setFixedSize(300, 50)
        self.close_button.clicked.connect(self.close)
        layout.addWidget(self.close_button, alignment=QtCore.Qt.AlignHCenter)
