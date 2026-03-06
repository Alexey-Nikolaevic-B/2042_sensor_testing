from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR


class ColDetails(QWidget):
    save_requested    = pyqtSignal(dict)   # sensor_data
    save_as_requested = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_details.ui", self)
        self._sensor_data: dict | None = None
        self._setup_styles()
        self._connect_signals()


    def load_sensor(self, sensor_data: dict):
        self._sensor_data = sensor_data
        self._load_image(sensor_data.get("image_path", ""))
        self.lbl_sensor_name.setText(sensor_data.get("name", ""))
        self.lbl_description.setText(sensor_data.get("description", ""))
        self._load_params(sensor_data.get("params", {}))

    def clear(self):
        self._sensor_data = None
        self.lbl_sensor_image.clear()
        self.lbl_sensor_image.setText("no sensor image")
        self.lbl_sensor_name.setText("—")
        self.lbl_description.setText("")
        self._clear_params()


    def _load_image(self, path: str):
        if path:
            px = QPixmap(path)
            if not px.isNull():
                self.lbl_sensor_image.setPixmap(
                    px.scaled(
                        self.lbl_sensor_image.width(),
                        self.lbl_sensor_image.height(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
                return
        self.lbl_sensor_image.clear()
        self.lbl_sensor_image.setText("no sensor image")

    def _load_params(self, params: dict):
        self._clear_params()
        layout = self.grp_params.layout()
        for key, value in params.items():
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 2, 0, 2)
            h.setSpacing(8)
            lbl_key = QLabel(str(key))
            lbl_key.setStyleSheet(
                f"color: {Colors.TEXT_SECONDARY}; font-size: 12px; min-width: 120px;"
            )
            lbl_val = QLabel(str(value))
            lbl_val.setStyleSheet(
                f"color: {Colors.TEXT_PRIMARY}; font-size: 12px;"
            )
            lbl_val.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            h.addWidget(lbl_key)
            h.addWidget(lbl_val)
            layout.addWidget(row)

    def _clear_params(self):
        layout = self.grp_params.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def _connect_signals(self):
        self.btn_save.clicked.connect(
            lambda: self.save_requested.emit(self._sensor_data or {})
        )
        self.btn_save_as.clicked.connect(
            lambda: self.save_as_requested.emit(self._sensor_data or {})
        )

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_sensor_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
            }}
            QLabel#lbl_sensor_name {{
                color: {Colors.TEXT_WHITE};
                font-size: 16px;
                font-weight: bold;
            }}
            QLabel#lbl_description {{
                color: {Colors.TEXT_SECONDARY};
                font-size: 12px;
            }}
            QScrollArea {{ border: none; background-color: transparent; }}
            QWidget#scroll_info_contents {{ background-color: transparent; }}
            {Styles.GROUP_BOX}
            {Styles.SCROLLBAR}
        """)
        for btn, icon, style in [
            (self.btn_save,    Icons.SAVE(),  Styles.BUTTON_DEFAULT),
            (self.btn_save_as, Icons.SAVE(),  Styles.BUTTON_DEFAULT),
            (self.btn_edit,    Icons.EDIT(),  Styles.BUTTON_ICON),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(style)
