import os
import shutil

from PyQt5.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QFileDialog, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR

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
        self.setFixedSize(680, 520)

        self._edit_mode    = sensor_data is not None
        self._sensor_data  = sensor_data or {}
        self._sensor_id    = self._sensor_data.get("id")
        self._image_source = ""
        self._sdf_source   = ""
        self._params:dict  = {}
        self._detected_type: str | None = None

        uic.loadUi(f"{QT_DIR}/add_sensor_dialog.ui", self)
        self._setup_styles()
        self._connect_signals()
        self.params_layout.addStretch()

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

    def _connect_signals(self):
        self.btn_dialog_close.clicked.connect(self.reject)
        self.btn_cancel.clicked.connect(self.reject)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_browse_sdf.clicked.connect(self._pick_sdf)
        self.image_container.mousePressEvent = lambda _: self._pick_image()

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
            from src.sensors import REGISTRY
            SensorClass = REGISTRY.get(sensor_type)
            if SensorClass:
                params = SensorClass(path).get_params()
                self._populate_params(params)
        except Exception:
            self._populate_params({})

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
            image_dest = os.path.join(asset_dir, f"image{ext}")
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
            "description": self._sensor_data.get("description", ""),
            "params":      dict(self._params),
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


    def _setup_styles(self):
        C = Colors
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {C.BG_CARD};
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 6px;
            }}
            QWidget#dialog_titlebar {{
                background-color: {C.BG_TOOLBAR};
                border-bottom: 1px solid {C.BORDER};
            }}
            QLabel#dialog_title {{
                color: {C.TEXT_WHITE};
                font-size: 14px;
                font-weight: bold;
            }}
            QPushButton#btn_dialog_close {{
                background: transparent; border: none; border-radius: 4px;
            }}
            QPushButton#btn_dialog_close:hover {{ background-color: {C.BG_CARD_HOVER}; }}
            QLabel#lbl_section_image, QLabel#lbl_section_name,
            QLabel#lbl_section_sdf,   QLabel#lbl_section_type,
            QLabel#lbl_section_params {{
                color: {C.TEXT_SECONDARY};
                font-size: 11px;
                font-weight: bold;
                background: transparent;
            }}
            QWidget#image_container {{
                background-color: {C.BG_IMAGE};
                border: 1px solid {C.BORDER};
                border-radius: 4px;
            }}
            QLabel#image_label {{
                color: {C.TEXT_MUTED}; font-size: 12px; background: transparent;
            }}
            QLabel#sdf_label {{
                color: {C.TEXT_SECONDARY}; font-size: 12px;
            }}
            QLineEdit#input_name, QLineEdit#field_input {{
                background-color: {C.BG_INPUT};
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_PRIMARY};
                padding: 6px 10px;
                font-size: 13px;
            }}
            QLineEdit#input_name:focus, QLineEdit#field_input:focus {{
                border-color: {C.ACCENT};
            }}
            QScrollArea#params_scroll {{
                background-color: {C.BG_COLUMN};
                border: 1px solid {C.BORDER};
                border-radius: 4px;
            }}
            QWidget#params_contents {{ background-color: {C.BG_COLUMN}; }}
            QFrame#param_row {{
                background-color: transparent;
                border-bottom: 1px solid {C.BORDER};
            }}
            QLabel#param_key {{
                color: {C.TEXT_SECONDARY}; font-size: 12px; background: transparent;
            }}
            QLabel#params_placeholder {{
                color: {C.TEXT_MUTED}; font-size: 11px;
                padding: 8px; background: transparent;
            }}
            QWidget#wt_buttons {{
                background-color: {C.BG_TOOLBAR};
                border-top: 1px solid {C.BORDER};
            }}
            QPushButton#btn_browse_sdf {{
                background-color: {C.BG_CARD};
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_PRIMARY};
                font-size: 12px;
                padding: 0 12px;
            }}
            QPushButton#btn_browse_sdf:hover {{ background-color: {C.BG_CARD_HOVER}; }}
            QPushButton#btn_cancel {{
                background-color: transparent;
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_SECONDARY};
                font-size: 13px;
                padding: 0 16px;
            }}
            QPushButton#btn_cancel:hover {{ background-color: {C.BG_CARD_HOVER}; }}
            QPushButton#btn_save {{
                background-color: {C.ACCENT_DIM};
                border: 1px solid {C.ACCENT};
                border-radius: 4px;
                color: {C.ACCENT};
                font-size: 13px;
                font-weight: bold;
                padding: 0 20px;
            }}
            QPushButton#btn_save:hover {{ background-color: #1e3d50; }}
            QPushButton#btn_save:pressed {{ background-color: {C.BG_APP}; }}
            {Styles.SCROLLBAR}
        """)
        self.btn_dialog_close.setIcon(Icons.CLOSE())
        self.btn_dialog_close.setIconSize(Layout.ICON_SIZE_SM)
        # Initial state: dot muted until SDF loaded
        self._set_detected_type(None)