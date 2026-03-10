from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QFrame, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .dialog_add_sensor import AddSensorDialog

DESCRIPTION_STYLE = f"""
    QScrollArea {{
        border: none;
        border-top: 1px solid {Colors.BORDER};
        border-bottom: 1px solid {Colors.BORDER};
        background-color: {Colors.BG_TOOLBAR};
    }}
    QWidget#scroll_description_contents {{
        background-color: {Colors.BG_TOOLBAR};
    }}
    QLabel {{
        color: {Colors.TEXT_SECONDARY};
        font-size: 12px;
        background-color: transparent;
    }}
    {Styles.SCROLLBAR}
"""

IMAGE_H   = 200
TOOLBAR_H = 44
NAME_H    = 36
DESC_H    = 72


class ColDetails(QWidget):
    save_requested    = pyqtSignal(dict)
    save_as_requested = pyqtSignal(dict)
    sensor_updated    = pyqtSignal(dict)   # emitted after edit dialog saves

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_2.ui", self)
        self._sensor_data: dict | None = None
        self._enforce_heights()
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
        self.lbl_sensor_name.setText("")
        self.lbl_description.setText("")
        self._clear_params()

    def _enforce_heights(self):
        self.lbl_sensor_image.setFixedHeight(IMAGE_H)
        self.wt_toolbar.setFixedHeight(TOOLBAR_H)
        self.lbl_sensor_name.setFixedHeight(NAME_H)
        self.scroll_description.setFixedHeight(DESC_H)

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
        layout = self.scroll_params_contents.layout()
        for key, value in params.items():
            row = QFrame()
            row.setStyleSheet(
                f"QFrame {{ border-bottom: 1px solid {Colors.BORDER}; background: transparent; }}"
            )
            h = QHBoxLayout(row)
            h.setContentsMargins(12, 6, 12, 6)
            h.setSpacing(12)
            lbl_key = QLabel(str(key))
            lbl_key.setStyleSheet(
                f"color: {Colors.TEXT_SECONDARY}; font-size: 12px;"
                f" min-width: 110px; max-width: 110px; background: transparent; border: none;"
            )
            lbl_val = QLabel(str(value))
            lbl_val.setStyleSheet(
                f"color: {Colors.TEXT_PRIMARY}; font-size: 12px; background: transparent; border: none;"
            )
            lbl_val.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            lbl_val.setWordWrap(True)
            h.addWidget(lbl_key)
            h.addWidget(lbl_val)
            layout.addWidget(row)

        filler = QWidget()
        filler.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        filler.setStyleSheet(f"background-color: transparent;")
        layout.addWidget(filler)

    def _clear_params(self):
        layout = self.scroll_params_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def _connect_signals(self):
        self.btn_edit.clicked.connect(self._on_edit)

    def _on_edit(self):
        if not self._sensor_data:
            return
        dlg = AddSensorDialog(parent=self, sensor_data=self._sensor_data)
        dlg.sensor_saved.connect(self._on_edit_saved)
        dlg.exec_()

    def _on_edit_saved(self, sensor_dict: dict):
        self.sensor_updated.emit(sensor_dict)
        # Refresh the column with new data immediately
        self.load_sensor(sensor_dict)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget#ColDetails {{ background-color: {Colors.BG_COLUMN}; }}
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
                font-size: 15px;
                font-weight: bold;
                padding: 0 12px;
            }}
            QScrollArea#scroll_params {{
                border: none;
                background-color: transparent;
            }}
            QWidget#scroll_params_contents {{
                background-color: transparent;
            }}
            {Styles.SCROLLBAR}
        """)
        self.scroll_description.setStyleSheet(DESCRIPTION_STYLE)
        for btn, icon, style in [
            (self.btn_edit,    Icons.EDIT(), Styles.BUTTON_ICON),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(style)