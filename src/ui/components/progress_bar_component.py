from PyQt5 import QtWidgets


class MyProgressBar(QtWidgets.QProgressBar):
    def __init__(self, parent=None):
        super(MyProgressBar, self).__init__(parent)
        self.setupUi()

    def setupUi(self):
        # Устанавливаем в режим индикатора активности
        self.setMinimum(0)
        self.setMaximum(0)  # Установка диапазона 0-0 включает режим зацикленной загрузки

        self.setStyleSheet("""
            QProgressBar {
                border: 2px solid #545659;
                border-radius: 10px;
                background: #ecf0f1;
                text-align: center;
                color: #2c3e50;
                font: bold 14px;
                padding: 0px 5px;
            }
            QProgressBar::chunk {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #F0D000, stop:1 #F0D000
                );
                border-radius: 10px;
            }
        """)
