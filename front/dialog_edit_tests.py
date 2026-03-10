import os
import shutil

from PyQt5.QtWidgets import QDialog, QWidget, QLabel, QSizePolicy, QScrollArea, QFrame
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QColor
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR

ASSETS_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "sensors")
IMAGE_W, IMAGE_H = 320, 160


class TestMetaRow(QWidget):
    clicked = pyqtSignal(str)

    def __init__(self, test: dict, parent=None):
        super().__init__(parent)
        self.func_name = test["func_name"]
        self.missing   = test.get("missing", False)
        self._selected = False

        self.setFixedHeight(54)
        self.setCursor(Qt.PointingHandCursor)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setObjectName("TestMetaRow")

        from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout
        h = QHBoxLayout(self)
        h.setContentsMargins(12, 0, 12, 0)
        h.setSpacing(8)

        self._bar = QWidget()
        self._bar.setFixedWidth(3)
        self._bar.setFixedHeight(32)
        self._bar.setAttribute(Qt.WA_StyledBackground, True)
        h.addWidget(self._bar)

        col = QVBoxLayout()
        col.setSpacing(2)
        self._lbl_display = QLabel(test.get("display_name") or test["func_name"])
        self._lbl_display.setObjectName("row_display")
        self._lbl_func = QLabel(test["func_name"])
        self._lbl_func.setObjectName("row_func")
        col.addWidget(self._lbl_display)
        col.addWidget(self._lbl_func)
        h.addLayout(col)
        h.addStretch()

        if self.missing:
            warn = QLabel("not in core")
            warn.setObjectName("row_warn")
            h.addWidget(warn)

        self._refresh_style()

    def set_selected(self, selected: bool):
        self._selected = selected
        self._refresh_style()

    def update_display_name(self, name: str):
        self._lbl_display.setText(name or self.func_name)

    def _refresh_style(self):
        bar_color = Colors.STATUS_YELLOW if self.missing else Colors.ACCENT
        bg        = Colors.ACCENT_BLUE_TEXT if self._selected else "transparent"
        self._bar.setStyleSheet(f"background-color: {bar_color};")
        self.setStyleSheet(f"""
            QWidget#TestMetaRow {{
                background-color: {bg};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#TestMetaRow:hover {{
                background-color: {Colors.TEXT_PRIMARY};
            }}
        """)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.func_name)
        super().mousePressEvent(event)


class EditTestsDialog(QDialog):

    tests_saved = pyqtSignal()

    def __init__(self, sensor_id: str, sensor_name: str,
                 tests: list[dict], parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_StyledBackground, True)

        self._sensor_id   = sensor_id
        self._sensor_name = sensor_name
        self._tests: dict[str, dict] = {t["func_name"]: dict(t) for t in tests}
        self._selected: str | None   = None
        self._rows: dict[str, TestMetaRow] = {}

        uic.loadUi(f"{QT_DIR}/edit_tests_dialog.ui", self)
        self._setup_styles()
        self._populate_list(tests)
        self._connect_signals()

        self.wt_editor.setVisible(False)
        self.lbl_empty.setVisible(True)

        if tests:
            self._select(tests[0]["func_name"])

    def _populate_list(self, tests: list[dict]):
        layout = self.scroll_tests_list_contents.layout()
        for test in tests:
            row = TestMetaRow(test, parent=self.scroll_tests_list_contents)
            row.clicked.connect(self._select)
            layout.addWidget(row)
            self._rows[test["func_name"]] = row
        layout.addStretch(1)

    def _select(self, func_name: str):
        if self._selected and self._selected in self._rows:
            self._rows[self._selected].set_selected(False)

        self._selected = func_name
        self._rows[func_name].set_selected(True)
        test = self._tests[func_name]

        self.lbl_empty.setVisible(False)
        self.wt_editor.setVisible(True)

        self.lbl_missing_warn.setVisible(test.get("missing", False))

        self.input_display_name.blockSignals(True)
        self.input_description.blockSignals(True)
        self.input_display_name.setText(test.get("display_name") or "")
        self.input_func_name.setText(func_name)
        self.input_description.setPlainText(test.get("description") or "")
        self.input_display_name.blockSignals(False)
        self.input_description.blockSignals(False)

        self._load_image(test.get("image_path", ""))

    def _load_image(self, path: str):
        if path and os.path.isfile(path):
            px = QPixmap(path)
            if not px.isNull():
                self.lbl_image.setPixmap(self._crop(px))
                self.lbl_image.setText("")
                return
        self.lbl_image.clear()
        self.lbl_image.setText("Click to choose image")

    def _pick_image(self):
        if not self._selected:
            return
        from PyQt5.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(
            self, "Choose test image", "",
            "Images (*.png *.jpg *.jpeg *.bmp *.webp)"
        )
        if not path:
            return
        self._tests[self._selected]["image_path"] = path
        self._load_image(path)

    def _on_name_changed(self, text: str):
        if self._selected:
            self._tests[self._selected]["display_name"] = text
            self._rows[self._selected].update_display_name(text)

    def _on_desc_changed(self):
        if self._selected:
            self._tests[self._selected]["description"] = self.input_description.toPlainText()

    def _on_save_all(self):
        from .logic_sensor_repository import SensorRepository
        repo = SensorRepository.instance()

        asset_dir = os.path.join(ASSETS_DIR, self._sensor_name, "tests")
        os.makedirs(asset_dir, exist_ok=True)

        for func_name, test in self._tests.items():
            src    = test.get("image_path", "")
            stored = src
            if src and os.path.isfile(src) and not src.startswith(asset_dir):
                ext  = os.path.splitext(src)[1].lower() or ".png"
                dest = os.path.join(asset_dir, f"{func_name}{ext}")
                px   = QPixmap(src)
                if not px.isNull():
                    self._crop(px).save(dest)
                else:
                    shutil.copy2(src, dest)
                stored = dest

            repo.save_test_meta(
                sensor_id    = self._sensor_id,
                func_name    = func_name,
                display_name = test.get("display_name") or func_name,
                description  = test.get("description") or "",
                image_path   = stored,
            )

        self.tests_saved.emit()
        self.accept()

    def _crop(self, px: QPixmap) -> QPixmap:
        scaled = px.scaled(IMAGE_W, IMAGE_H, Qt.KeepAspectRatioByExpanding,
                           Qt.SmoothTransformation)
        x = (scaled.width()  - IMAGE_W) // 2
        y = (scaled.height() - IMAGE_H) // 2
        return scaled.copy(x, y, IMAGE_W, IMAGE_H)

    def _connect_signals(self):
        self.btn_close.clicked.connect(self.reject)
        self.btn_cancel.clicked.connect(self.reject)
        self.btn_save.clicked.connect(self._on_save_all)
        self.wt_image_container.mousePressEvent = lambda _: self._pick_image()
        self.input_display_name.textChanged.connect(self._on_name_changed)
        self.input_description.textChanged.connect(self._on_desc_changed)

    def _setup_styles(self):
        C = Colors
        self.setStyleSheet(f"""
            QWidget#wt_titlebar {{
                background-color: {C.BG_COLUMN};
                border-bottom: 1px solid {C.BORDER};
            }}
            QLabel#lbl_title {{
                color: {C.TEXT_WHITE};
                font-size: 14px;
                font-weight: bold;
                background: transparent;
            }}
            QWidget#wt_list_panel {{
                background-color: {C.BG_COLUMN};
                border-right: 1px solid {C.BORDER};
            }}
            QLabel#row_display {{
                color: {C.TEXT_BLACK};
                font-size: 13px;
                background: transparent;
            }}
            QLabel#row_func {{
                color: {C.TEXT_MUTED};
                font-size: 10px;
                background: transparent;
            }}
            QLabel#row_warn {{
                color: {C.STATUS_YELLOW};
                font-size: 10px;
                background: transparent;
            }}
            QWidget#wt_editor_panel {{
                background-color: transparent;
            }}
            QLabel#lbl_func_badge {{
                color: {C.TEXT_MUTED};
                font-size: 11px;
                font-family: monospace;
                background: transparent;
            }}
            QLabel#lbl_empty {{
                color: {C.TEXT_MUTED};
                font-size: 13px;
                background: transparent;
            }}
            QLabel#lbl_section_image,
            QLabel#lbl_section_name,
            QLabel#lbl_section_func,
            QLabel#lbl_section_desc {{
                color: {C.TEXT_MUTED};
                font-size: 10px;
                font-weight: bold;
                letter-spacing: 1px;
                background: transparent;
            }}
            QWidget#wt_image_container {{
                background-color: {C.BG_IMAGE};
                border: 1px solid {C.BORDER};
                border-radius: 4px;
            }}
            QLabel#lbl_image {{
                color: {C.TEXT_MUTED};
                font-size: 12px;
                background: transparent;
            }}
            QLineEdit {{
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_BLACK};
                padding: 6px 10px;
                font-size: 13px;
            }}
            QLineEdit:focus {{ border-color: {C.ACCENT}; }}
            QLineEdit[readOnly="true"] {{
                color: {C.TEXT_BLACK};
                border-color: {C.BORDER};
            }}
            QPlainTextEdit {{
            background: transparent;
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_BLACK};
                padding: 6px 10px;
                font-size: 13px;
            }}
            QPlainTextEdit:focus {{ border-color: {C.ACCENT}; }}
            QLabel#lbl_missing_warn {{
                color: {C.STATUS_YELLOW};
                background-color: #2a2000;
                border: 1px solid {C.STATUS_YELLOW};
                border-radius: 4px;
                padding: 8px;
                font-size: 12px;
            }}
            QWidget#wt_footer {{
                background-color: {C.BG_COLUMN};
                border-top: 1px solid {C.BORDER};
            }}
            QPushButton#btn_cancel {{
                background: transparent;
                border: 1px solid {C.BORDER_LIGHT};
                border-radius: 4px;
                color: {C.TEXT_MUTED};
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
            QPushButton#btn_close {{
                background: transparent;
                border: none;
                border-radius: 4px;
            }}
            QPushButton#btn_close:hover {{ background-color: {C.BG_CARD_HOVER}; }}

            {Styles.SCROLLBAR}
        """)

        self.btn_close.setIcon(Icons.CLOSE())
        self.btn_close.setIconSize(Layout.ICON_SIZE_SM)
        self.lbl_title.setText(f"Edit tests")