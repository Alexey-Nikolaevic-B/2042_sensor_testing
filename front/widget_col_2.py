from PyQt5.QtWidgets import QWidget, QLabel, QHBoxLayout, QFrame, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .dialog_add_sensor import AddSensorDialog


_fmt_value = lambda v: "\n".join(f"- {x}" for x in v) if isinstance(v, list) else str(v)


class ColDetails(QWidget):
    save_requested    = pyqtSignal(dict)
    save_as_requested = pyqtSignal(dict)
    sensor_updated    = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_2.ui", self)
        self._sensor_data: dict | None = None
        self._setup_heights()
        self._setup_styles()
        self._connect_signals()

    # ── public API ────────────────────────────────────────────────────────────

    def load_sensor(self, sensor_data: dict):
        self._sensor_data = sensor_data
        self._load_image(sensor_data.get("image_path", ""))
        self.lbl_sensor_name.setText(sensor_data.get("name", ""))
        self.lbl_description.setText(sensor_data.get("description", ""))
        
        # Create params with topics as separate entries
        params_with_topics = self._create_params_with_topics(sensor_data)
        self._load_params(params_with_topics)

    def clear(self):
        self._sensor_data = None
        self.lbl_sensor_image.clear()
        self.lbl_sensor_image.setText("no sensor image")
        self.lbl_sensor_name.setText("")
        self.lbl_description.setText("")
        self._clear_params()

    # ── helper method to create params with topics ────────────────────────────

    def _create_params_with_topics(self, sensor_data: dict) -> dict:
        """
        Create a params dictionary with each topic as a separate entry
        (topic_1, topic_2, etc.), followed by the original params.
        """
        topics = sensor_data.get("topics", [])
        original_params = sensor_data.get("params", {})
        
        # Create new dict with topics as separate entries
        params_with_topics = {}
        
        # Add each topic as a separate entry
        for i, topic in enumerate(topics, 1):
            params_with_topics[f"topic_{i}"] = topic
        
        # If no topics, add a placeholder
        if not topics:
            params_with_topics["topics"] = "No topics defined"
        
        # Add all original params
        params_with_topics.update(original_params)
        
        return params_with_topics

    # ── slots ─────────────────────────────────────────────────────────────────

    def _on_edit(self):
        if not self._sensor_data:
            return
        dlg = AddSensorDialog(parent=self, sensor_data=self._sensor_data)
        dlg.sensor_saved.connect(self._on_edit_saved)
        dlg.exec_()

    def _on_edit_saved(self, sensor_dict: dict):
        self.sensor_updated.emit(sensor_dict)
        self.load_sensor(sensor_dict)

    # ── helpers ───────────────────────────────────────────────────────────────

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
            lbl_val = QLabel(_fmt_value(value))
            lbl_val.setStyleSheet(
                f"color: {Colors.TEXT_PRIMARY}; font-size: 12px; background: transparent; border: none;"
            )
            lbl_val.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            lbl_val.setWordWrap(True)
            lbl_val.setTextInteractionFlags(Qt.TextSelectableByMouse)
            
            h.addWidget(lbl_key)
            h.addWidget(lbl_val)
            layout.addWidget(row)

        filler = QWidget()
        filler.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        filler.setStyleSheet("background-color: transparent;")
        layout.addWidget(filler)

    def _clear_params(self):
        layout = self.scroll_params_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    # ── setup ─────────────────────────────────────────────────────────────────

    def _setup_heights(self):
        self.lbl_sensor_image.setFixedHeight(Layout.IMAGE_H)
        self.wt_toolbar.setFixedHeight(Layout.TOOLBAR_H)
        self.lbl_sensor_name.setFixedHeight(Layout.NAME_H)
        self.scroll_description.setFixedHeight(Layout.DESC_H)

    def _connect_signals(self):
        self.btn_edit.clicked.connect(self._on_edit)

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
        self.scroll_description.setStyleSheet(
            Styles.DESCRIPTION_AREA
        )
        self.btn_edit.setIcon(Icons.EDIT())
        self.btn_edit.setIconSize(Layout.ICON_SIZE_MD)
        self.btn_edit.setStyleSheet(Styles.BUTTON_ICON)