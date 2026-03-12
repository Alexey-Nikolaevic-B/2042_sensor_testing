import os
import shutil

from PyQt5.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QFileDialog, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR, LightColors as LC, LightStyles as LS

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "sensors")
IMAGE_W, IMAGE_H = 300, 200


class AddSensorDialog(QDialog):

    sensor_saved = pyqtSignal(dict)

    def __init__(self, parent=None, sensor_data: dict | None = None, core=None):
        """
        Pass sensor_data to open in edit mode (fields pre-filled).
        Pass None  to open in add mode (empty fields).
        Pass core  to enable SDF type auto-detection.
        """
        super().__init__(parent)
        self._core = core
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setFixedSize(880, 520)

        self._edit_mode    = sensor_data is not None
        self._sensor_data  = sensor_data or {}
        self._sensor_id    = self._sensor_data.get("id")
        self._image_source = ""
        self._sdf_source   = ""
        self._params:dict  = {}
        self._topic_rows:list = []
        self._detected_type: str | None = None

        uic.loadUi(f"{QT_DIR}/add_sensor_dialog.ui", self)
        self._setup_styles()
        self._connect_signals()
        self.params_layout.addStretch()
        self.topics_layout.addStretch()

        if self._edit_mode:
            self._prefill()

    def _prefill(self):
        d = self._sensor_data
        self.dialog_title.setText("Edit sensor")
        self.btn_save.setText("Save changes")

        self.input_name.setText(d.get("name", ""))

        img = d.get("image_path", "")
        if img and os.path.exists(img):
            self._image_source = img
            px = QPixmap(img)
            if not px.isNull():
                self.image_label.setPixmap(self._center_crop(px, IMAGE_W, IMAGE_H))
                self.image_label.setText("")

        sdf = d.get("sdf_path", "")
        if sdf:
            self._sdf_source = sdf
            self.sdf_label.setText(os.path.basename(sdf))

        sensor_type = d.get("type", "unknown")
        self._set_detected_type(sensor_type)
        self._populate_params(d.get("params", {}))
        self.input_description.setPlainText(d.get("description", ""))
        for t in d.get("topics", []):
            self._add_topic_row(t)

    def _connect_signals(self):
        self.btn_dialog_close.clicked.connect(self.reject)
        self.btn_cancel.clicked.connect(self.reject)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_browse_sdf.clicked.connect(self._pick_sdf)
        self.image_container.mousePressEvent = lambda _: self._pick_image()
        self.btn_add_topic.clicked.connect(lambda: self._add_topic_row(''))

    def _pick_image(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose sensor image", "",
            "Images (*.png *.jpg *.jpeg *.bmp *.webp)"
        )
        if not path:
            return
        self._image_source = path
        px = QPixmap(path)
        if not px.isNull():
            self.image_label.setPixmap(self._center_crop(px, IMAGE_W, IMAGE_H))
            self.image_label.setText("")

    def _pick_sdf(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose SDF file", "",
            "SDF files (*.sdf);;All files (*)"
        )
        if not path:
            return
        self._sdf_source = path
        self.sdf_label.setText(os.path.basename(path))
        self._on_sdf_selected(path)

    def _on_sdf_selected(self, path: str):
        if self._core is None:
            self._set_detected_type("unknown")
            self._populate_params({})
            return

        sensor_type = self._core.detect_sensor_type(path)

        if sensor_type is None:
            self._set_detected_type("unknown")
            self._populate_params({})
            return

        self._set_detected_type(sensor_type)

        try:
            from src.sensors.sensor import Sensor as SensorModel
            instance = SensorModel(
                sensor_type = sensor_type,
                sensor_name = "",
                sdf_path    = path,
            )
            params = self._core.read_sensor_params(instance)
            self._populate_params(params)
        except Exception:
            self._populate_params({})

        # Auto-detect topics from SDF
        for row in list(self._topic_rows):
            self._remove_topic_row(row)
        try:
            from sensor import detect_topics_from_sdf
        except ImportError:
            try:
                from src.sensors.sensor import detect_topics_from_sdf
            except ImportError:
                detect_topics_from_sdf = None
        if detect_topics_from_sdf:
            for t in (detect_topics_from_sdf(path) or []):
                self._add_topic_row(t)

    def _set_detected_type(self, sensor_type: str | None):
        self._detected_type = sensor_type
        if sensor_type != "unknown":
            color = Colors.STATUS_GREEN
            display = sensor_type
        else:
            color = Colors.STATUS_YELLOW
            display = "unknown"

        dot = getattr(self, "lbl_type_dot", None)
        if dot is not None:
            dot.setStyleSheet(f"color: {color}; background: transparent;")

        self.lbl_detected_type.setText(display)
        self.lbl_detected_type.setStyleSheet(
            f"color: {color}; background: transparent;"
        )

    def _populate_params(self, params: dict):
        while self.params_layout.count() > 1:
            item = self.params_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._params = dict(params)
        if params:
            for key, value in params.items():
                self.params_layout.insertWidget(
                    self.params_layout.count() - 1,
                    self._make_param_row(key, str(value))
                )
        else:
            lbl = QLabel("No parameters detected.\nWill populate after backend SDF validation.")
            lbl.setObjectName("params_placeholder")
            lbl.setWordWrap(True)
            self.params_layout.insertWidget(0, lbl)

    def _make_param_row(self, key: str, value: str) -> QWidget:
        row = QFrame()
        row.setObjectName("param_row")
        h = QHBoxLayout(row)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(12)
        lbl_key = QLabel(key)
        lbl_key.setObjectName("param_key")
        lbl_key.setFixedWidth(110)
        edit_val = QLineEdit(value)
        edit_val.setObjectName("field_input")
        edit_val.textChanged.connect(lambda v, k=key: self._params.update({k: v}))
        h.addWidget(lbl_key)
        h.addWidget(edit_val)
        return row

    def _on_save(self):
        name = self.input_name.text().strip()
        if not name:
            self._flash_error(self.input_name)
            return
        if not self._sdf_source:
            self._flash_error(self.sdf_label)
            return
        if self._detected_type == "unknown":
            self._flash_error(self.lbl_detected_type)
            return
        

        asset_dir = os.path.join(ASSETS_DIR, name)
        os.makedirs(asset_dir, exist_ok=True)

        sdf_dest = os.path.join(asset_dir, "model.sdf")
        if os.path.abspath(self._sdf_source) != os.path.abspath(sdf_dest):
            shutil.copy2(self._sdf_source, sdf_dest)

        image_dest = self._sensor_data.get("image_path", "")
        if self._image_source:
            ext = os.path.splitext(self._image_source)[1].lower() or ".png"
            image_dest = os.path.abspath(os.path.join(asset_dir, f"image{ext}"))
            if os.path.abspath(self._image_source) != os.path.abspath(image_dest):
                px = QPixmap(self._image_source)
                if not px.isNull():
                    self._center_crop(px, IMAGE_W, IMAGE_H).save(image_dest)
                else:
                    shutil.copy2(self._image_source, image_dest)

        result = {
            "name":        name,
            "type":        self._detected_type or "unknown",
            "sdf_path":    sdf_dest,
            "image_path":  image_dest,
            "description": self.input_description.toPlainText().strip(),
            "params":      dict(self._params),
            "topics":      self._collect_topics(),
        }
        if self._sensor_id:
            result["id"] = self._sensor_id

        self.sensor_saved.emit(result)
        self.accept()


    @staticmethod
    def _center_crop(px: QPixmap, w: int, h: int) -> QPixmap:
        scaled = px.scaled(w, h, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        x = (scaled.width()  - w) // 2
        y = (scaled.height() - h) // 2
        return scaled.copy(x, y, w, h)

    def _flash_error(self, widget: QWidget):
        orig = widget.styleSheet()
        widget.setStyleSheet(orig + "border: 1px solid #ef4444;")
        QTimer.singleShot(1200, lambda: widget.setStyleSheet(orig))


    def _collect_topics(self) -> list:
        result = []
        for row in self._topic_rows:
            edit = row.findChild(QLineEdit)
            if edit:
                t = edit.text().strip()
                if t:
                    result.append(t)
        return result

    def _add_topic_row(self, topic: str = ''):
        row = QFrame()
        row.setObjectName('topic_row')
        h = QHBoxLayout(row)
        h.setContentsMargins(4, 2, 4, 2)
        h.setSpacing(4)
        edit = QLineEdit(topic)
        edit.setObjectName('field_input')
        edit.setPlaceholderText('/topic/name')
        btn = QPushButton('✕')
        btn.setObjectName('btn_remove_topic')
        btn.setFixedSize(22, 22)
        btn.setStyleSheet(
            f'QPushButton {{ background: transparent; border: none; color: {LC.TEXT_MUTED}; font-size: 12px; }}'
            f'QPushButton:hover {{ color: {LC.ERROR}; }}'
        )
        btn.clicked.connect(lambda: self._remove_topic_row(row))
        h.addWidget(edit)
        h.addWidget(btn)
        lay = self.topics_layout
        lay.insertWidget(lay.count() - 1, row)
        self._topic_rows.append(row)

    def _remove_topic_row(self, row):
        if row in self._topic_rows:
            self._topic_rows.remove(row)
        self.topics_layout.removeWidget(row)
        row.deleteLater()

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QDialog {{
                background: {LC.BG_PANEL};
            }}
            QWidget {{
                background: {LC.BG_PANEL};
                color: {LC.TEXT};
                font-size: 13px;
            }}
            QWidget#dialog_titlebar {{
                background: {LC.BG};
                border-bottom: 1px solid {LC.BORDER};
            }}
            QWidget#wt_buttons {{
                background: {LC.BG};
                border-top: 1px solid {LC.BORDER};
            }}
            QWidget#image_container {{
                background: {LC.BG_HOVER};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
            }}
            QScrollArea#params_scroll {{
                background: {LC.BG};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
            }}
            QWidget#params_contents {{ background: {LC.BG}; }}
            QFrame#param_row {{
                background: transparent;
                border-bottom: 1px solid {LC.BG_HOVER};
            }}
            QLabel#dialog_title {{
                color: {LC.TEXT};
                font-size: 14px;
                font-weight: 700;
                background: transparent;
            }}
            QLabel#lbl_section_image, QLabel#lbl_section_name,
            QLabel#lbl_section_sdf,   QLabel#lbl_section_type,
            QLabel#lbl_section_params, QLabel#lbl_section_description, QLabel#lbl_section_topics {{
                color: {LC.TEXT_SEC};
                font-size: 11px;
                font-weight: 600;
                letter-spacing: 0.5px;
                background: transparent;
            }}
            QLabel#image_label {{
                color: {LC.TEXT_MUTED};
                font-size: 12px;
                background: transparent;
            }}
            QLabel#sdf_label {{
                color: {LC.TEXT_SEC};
                font-size: 12px;
                background: transparent;
            }}
            QLabel#param_key {{
                color: {LC.TEXT_SEC};
                font-size: 12px;
                background: transparent;
            }}
            QLabel#params_placeholder {{
                color: {LC.TEXT_MUTED};
                font-size: 11px;
                padding: 8px;
                background: transparent;
            }}
            QLineEdit#input_name, QLineEdit#field_input {{
                background: {LC.BG_INPUT};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT};
                padding: 5px 8px;
                font-size: 13px;
            }}
            QLineEdit#input_name:focus, QLineEdit#field_input:focus {{
                border-color: {LC.ACCENT};
            }}
            QPlainTextEdit {{
                background: {LC.BG_INPUT};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT};
                padding: 5px 8px;
                font-size: 12px;
            }}
            QPlainTextEdit:focus {{ border-color: {LC.ACCENT}; }}
            QPushButton#btn_dialog_close {{
                background: transparent;
                border: none;
                border-radius: 4px;
            }}
            QPushButton#btn_dialog_close:hover {{ background: {LC.BG_HOVER}; }}
            QPushButton#btn_browse_sdf {{
                background: {LC.BG};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT};
                font-size: 12px;
                padding: 0 12px;
            }}
            QPushButton#btn_browse_sdf:hover {{ background: {LC.BG_HOVER}; }}
            QPushButton#btn_cancel {{
                background: transparent;
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT_SEC};
                font-size: 13px;
                padding: 0 16px;
            }}
            QPushButton#btn_cancel:hover {{ background: {LC.BG_HOVER}; color: {LC.TEXT}; }}
            QPushButton#btn_save {{
                background: {LC.ACCENT};
                border: none;
                border-radius: 4px;
                color: #ffffff;
                font-size: 13px;
                font-weight: 600;
                padding: 0 20px;
            }}
            QPushButton#btn_save:hover {{ background: {LC.ACCENT_HVR}; }}
            QPushButton#btn_save:pressed {{ background: #1e40af; }}
            QScrollArea#topics_scroll {{
                background: {LC.BG};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
            }}
            QWidget#topics_contents {{ background: {LC.BG}; }}
            QFrame#topic_row {{ background: transparent; border-bottom: 1px solid {LC.BG_HOVER}; }}
            QPushButton#btn_add_topic {{
                background: transparent;
                border: 1px dashed {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT_SEC};
                font-size: 12px;
                padding: 4px;
            }}
            QPushButton#btn_add_topic:hover {{ background: {LC.BG_HOVER}; color: {LC.TEXT}; }}
            {LS.SCROLLBAR}
        """)
        self.btn_dialog_close.setIcon(Icons.CLOSE())
        self.btn_dialog_close.setIconSize(Layout.ICON_SIZE_SM)
        self._set_detected_type(None)