from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import QSize

ICON_DIR = "./front/icon"
IMAGE_DIR = "./front/img"
QT_DIR = "./front/qt"


class Colors:
    BG_APP = "#111111"
    BG_TITLEBAR = "#161616"
    BG_COLUMN = "#161616"
    BG_TOOLBAR = "#1c1c1c"
    BG_CARD = "#1e1e1e"
    BG_CARD_HOVER = "#272727"
    BG_CARD_SEL = "#2e2e2e"
    BG_INPUT = "#1a1a1a"
    BG_LOG = "#0d0d0d"
    BG_IMAGE = "#000000"
    SPLITTER = "#2a2a2a"
    BORDER = "#252525"
    BORDER_LIGHT = "#333333"
    DIVIDER = "#1e1e1e"
    TEXT_PRIMARY = "#e0e0e0"
    TEXT_SECONDARY = "#888888"
    TEXT_MUTED = "#484848"
    TEXT_WHITE = "#ffffff"
    TEXT_BLACK = "#111111"
    ACCENT = "#4fc3f7"
    ACCENT_DIM = "#193040"
    ACCENT_HOVER = "#29b6f6"
    STATUS_GREEN = "#4caf50"
    STATUS_GRAY = "#757575"
    STATUS_RED = "#ef5350"
    STATUS_BLUE = "#42a5f5"
    STATUS_RUNNING = "#ec27ab"  # magenta
    STATUS_QUEUED = "#7c4dff"  # purple
    STATUS_YELLOW = "#ffb300"  # kept for back-compat
    # back-compat
    BG_WINDOW = BG_APP
    BG_SIDEBAR = BG_COLUMN
    BG_MAIN = BG_COLUMN
    BG_PRIMARY = BG_CARD
    BG_SECONDARY = BG_CARD
    BG_ELEVATED = BG_CARD_HOVER
    BG_HOVER = BG_CARD_HOVER
    BG_SELECTED = BG_CARD_SEL
    BORDER_FOCUS = BORDER_LIGHT
    ACCENT_BLUE = ACCENT
    ACCENT_BLUE_BG = ACCENT_DIM
    ACCENT_BLUE_BG_HVR = "#1e3d50"
    ACCENT_BLUE_TEXT = "#7dd3f7"
    ACCENT_GREEN = STATUS_GREEN
    ACCENT_GREEN_DARK = "#388e3c"
    ACCENT_GREEN_BG = "#1a2e1a"
    ACCENT_GREEN_BG_HVR = "#1f371f"
    ACCENT_RED = STATUS_RED
    STATUS_PASS = STATUS_GREEN
    STATUS_FAIL = STATUS_RED
    STATUS_PENDING = STATUS_YELLOW


class Styles:
    BUTTON_ICON = """
        QPushButton { background-color: transparent; border: none; padding: 4px; }
        QPushButton:hover { background-color: rgba(255,255,255,0.08); border-radius: 4px; }
        QPushButton:pressed { background-color: rgba(255,255,255,0.15); }
        QPushButton:disabled { opacity: 0.3; }
    """

    BUTTON_DEFAULT = f"""
        QPushButton {{
            border: 1px solid {Colors.BORDER_LIGHT};
            border-radius: 4px;
            background-color: {Colors.BG_CARD};
            color: {Colors.TEXT_PRIMARY};
            padding: 4px 10px;
        }}
        QPushButton:hover {{ background-color: {Colors.BG_CARD_HOVER}; }}
        QPushButton:pressed {{ background-color: {Colors.BG_APP}; }}
    """

    BUTTON_ACCENT = f"""
        QPushButton {{
            background-color: {Colors.ACCENT_DIM};
            border: 1px solid {Colors.ACCENT};
            border-radius: 4px;
            color: {Colors.ACCENT};
            padding: 4px 10px;
        }}
        QPushButton:hover {{ background-color: #1e3d50; }}
        QPushButton:pressed {{ background-color: {Colors.BG_APP}; }}
    """

    SCROLLBAR = f"""
        QScrollBar:vertical {{
            background: transparent; width: 5px; margin: 0;
        }}
        QScrollBar::handle:vertical {{
            background-color: {Colors.BORDER_LIGHT};
            border-radius: 2px; min-height: 20px;
        }}
        QScrollBar::handle:vertical:hover {{ background-color: #555; }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical
            {{ height: 0; background: transparent; }}
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
            {{ background: transparent; }}
        QScrollBar:horizontal {{ height: 0; background: transparent; }}
    """

    SCROLLBAR_HIDDEN = """
        QScrollBar:vertical { background: transparent; width: 0px; }
        QScrollBar::handle:vertical { background: transparent; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical
            { height: 0; background: transparent; }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
            { background: transparent; }
    """
    SCROLLBAR_THIN = SCROLLBAR

    GROUP_BOX = f"""
        QGroupBox {{
            border: 1px solid {Colors.BORDER};
            border-radius: 4px;
            font-weight: bold;
            color: {Colors.TEXT_SECONDARY};
            margin-top: 8px; padding-top: 8px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 10px;
            color: {Colors.TEXT_SECONDARY};
        }}
    """

    PROGRESS_BAR = f"""
        QProgressBar {{
            border: none;
            background-color: rgba(255,255,255,0.06);
            max-height: 4px; border-radius: 2px;
        }}
        QProgressBar::chunk {{
            background-color: {Colors.ACCENT}; border-radius: 2px;
        }}
    """

    LIST_WIDGET = f"""
        QListWidget {{
            border: none; background-color: transparent; outline: none;
        }}
        QListWidget::item {{
            border-bottom: 1px solid {Colors.DIVIDER};
            color: {Colors.TEXT_PRIMARY};
        }}
        QListWidget::item:hover {{ background-color: {Colors.BG_CARD_HOVER}; }}
        QListWidget::item:selected {{
            background-color: {Colors.BG_CARD_SEL};
            color: {Colors.TEXT_WHITE};
        }}
        QListWidget::item:focus {{ outline: none; border: none; }}
    """

    DESCRIPTION_AREA = (
        f"QScrollArea {{"
        f"  border: none;"
        f"  border-top: 1px solid {Colors.BORDER};"
        f"  border-bottom: 1px solid {Colors.BORDER};"
        f"  background-color: {Colors.BG_TOOLBAR};"
        f"}}"
        f"QWidget#scroll_description_contents {{"
        f"  background-color: {Colors.BG_TOOLBAR};"
        f"}}"
        f"QLabel {{"
        f"  color: {Colors.TEXT_SECONDARY};"
        f"  font-size: 12px;"
        f"  background-color: transparent;"
        f"}}"
    ) + SCROLLBAR

    # legacy aliases
    BUTTON_BLUE = BUTTON_ACCENT
    BUTTON_GREEN = BUTTON_DEFAULT
    LINE_EDIT = ""
    FILTER_FRAME = ""
    FILTER_BUTTON = BUTTON_DEFAULT
    CARD_NAME_LABEL = ""
    CARD_IMAGE_LABEL = ""


class Icons:
    _cache: dict = {}

    @classmethod
    def get(cls, f) -> QIcon:
        if f not in cls._cache:
            cls._cache[f] = QIcon(f"{ICON_DIR}/{f}")
        return cls._cache[f]

    @classmethod
    def MINIMIZE(cls):
        return cls.get("minimize.png")

    @classmethod
    def MAXIMIZE(cls):
        return cls.get("maximize.png")

    @classmethod
    def CLOSE(cls):
        return cls.get("close.png")

    @classmethod
    def MENU(cls):
        return cls.get("menu.png")

    @classmethod
    def SENSOR(cls):
        return cls.get("sensor.png")

    @classmethod
    def TEST_MENU(cls):
        return cls.get("test_menu.png")

    @classmethod
    def RUN(cls):
        return cls.get("run.png")

    @classmethod
    def STOP(cls):
        return cls.get("stop.png")

    @classmethod
    def RUN_ALL(cls):
        return cls.get("run_all.png")

    @classmethod
    def EXPORT(cls):
        return cls.get("export.png")

    @classmethod
    def FILTER(cls):
        return cls.get("filter.png")

    @classmethod
    def CLOSE_BLK(cls):
        return cls.get("close_black.png")

    @classmethod
    def CLEAR(cls):
        return cls.get("clear.png")

    @classmethod
    def COPY(cls):
        return cls.get("copy.png")

    @classmethod
    def SAVE(cls):
        return cls.get("save.png")

    @classmethod
    def ADD(cls):
        return cls.get("add.png")

    @classmethod
    def EDIT(cls):
        return cls.get("edit.png")

    @classmethod
    def SUCCESS(cls):
        return cls.get("success.png")

    @classmethod
    def FAIL(cls):
        return cls.get("fail.png")

    @classmethod
    def PENDING(cls):
        return cls.get("pending.png")

    @classmethod
    def RUNNING(cls):
        return cls.get("running.png")

    @classmethod
    def UNKNOWN(cls):
        return cls.get("unknown_status.png")

    @classmethod
    def TARGET_SENSOR(cls):
        return cls.get("target_sensor.png")

    @classmethod
    def OBSERVER(cls):
        return cls.get("observer.png")

    @classmethod
    def LOCK(cls):
        return cls.get("lock.png")

    @classmethod
    def STEP(cls):
        return cls.get("step.png")

    @classmethod
    def RUNNING_MOVIE(cls, label=None):
        from PyQt5.QtGui import QMovie

        movie = QMovie(f"{ICON_DIR}/pending.gif")
        if label is not None:
            movie.setScaledSize(label.size())
        return movie

    @classmethod
    def QUEUED(cls, label=None):
        """Returns a QMovie for the queued spinning animation."""
        from PyQt5.QtGui import QMovie

        movie = QMovie(f"{ICON_DIR}/pending.gif")
        if label is not None:
            movie.setScaledSize(Layout.ICON_SIZE_MD)
        return movie

    @classmethod
    def for_status(cls, status: str, is_running: bool = False) -> QIcon:
        if is_running:
            return cls.RUNNING()
        if status == "Passed":
            return cls.SUCCESS()
        if status == "Failed":
            return cls.FAIL()
        if status == "Idle":
            return cls.PENDING()
        if status == "Stopped":
            return cls.UNKNOWN()
        return cls.UNKNOWN()


class Layout:
    ICON_SIZE_SM = QSize(16, 16)
    ICON_SIZE_MD = QSize(20, 20)
    ICON_SIZE_LG = QSize(28, 28)
    # Column shared heights
    IMAGE_H = 200
    TOOLBAR_H = 44
    NAME_H = 36
    DESC_H = 100
    SENSOR_CELL_HEIGHT = 52
    TEST_ITEM_HEIGHT = 64
    TEST_LIST_MAX_H = 9999
    CARD_WIDTH = 350
    CARD_HEIGHT = 300
    CARD_H_GAP = 10
    CARD_V_GAP = 30


class LightColors:
    """Light-theme palette used by modal dialogs."""

    BG_PANEL = "#f0f0f0"
    BG = "#ffffff"
    BG_INPUT = "#fafafa"
    BG_HOVER = "#e8e8e8"
    BORDER = "#cccccc"
    TEXT = "#1a1a1a"
    TEXT_SEC = "#555555"
    TEXT_MUTED = "#999999"
    ACCENT = "#0078d4"
    ACCENT_HVR = "#106ebe"
    ACCENT_DIM = "#cce4f7"
    ERROR = "#d32f2f"


class LightStyles:
    """Light-theme styles used by modal dialogs."""

    BUTTON_ICON = """
        QPushButton { background-color: transparent; border: none; padding: 4px; }
        QPushButton:hover { background-color: rgba(0,0,0,0.08); border-radius: 4px; }
        QPushButton:pressed { background-color: rgba(0,0,0,0.15); }
        QPushButton:disabled { opacity: 0.4; }
    """

    BUTTON_DEFAULT = f"""
        QPushButton {{
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: #ffffff;
            color: #1a1a1a;
            padding: 4px 12px;
        }}
        QPushButton:hover {{ background-color: #e8e8e8; }}
        QPushButton:pressed {{ background-color: #d0d0d0; }}
    """

    BUTTON_PRIMARY = f"""
        QPushButton {{
            border: 1px solid #0078d4;
            border-radius: 4px;
            background-color: #0078d4;
            color: #ffffff;
            padding: 4px 12px;
            font-weight: 600;
        }}
        QPushButton:hover {{ background-color: #106ebe; }}
        QPushButton:pressed {{ background-color: #005a9e; }}
    """

    BUTTON_CANCEL = f"""
        QPushButton {{
            border: 1px solid #cccccc;
            border-radius: 4px;
            background-color: #f0f0f0;
            color: #555555;
            padding: 4px 12px;
        }}
        QPushButton:hover {{ background-color: #e0e0e0; }}
        QPushButton:pressed {{ background-color: #d0d0d0; }}
    """

    INPUT = f"""
        QLineEdit, QPlainTextEdit {{
            background-color: #ffffff;
            border: 1px solid #cccccc;
            border-radius: 3px;
            color: #1a1a1a;
            padding: 4px 8px;
            font-size: 13px;
        }}
        QLineEdit:focus, QPlainTextEdit:focus {{
            border-color: #0078d4;
        }}
    """

    SCROLLBAR = """
        QScrollBar:vertical {
            background: transparent; width: 5px; margin: 0;
        }
        QScrollBar::handle:vertical {
            background-color: #cccccc;
            border-radius: 2px; min-height: 20px;
        }
        QScrollBar::handle:vertical:hover { background-color: #aaaaaa; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical
            { height: 0; background: transparent; }
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
            { background: transparent; }
        QScrollBar:horizontal { height: 0; background: transparent; }
    """

    LIST_WIDGET = """
        QListWidget {
            border: 1px solid #cccccc;
            background-color: #ffffff;
            outline: none;
        }
        QListWidget::item {
            border-bottom: 1px solid #eeeeee;
            color: #1a1a1a;
            padding: 4px;
        }
        QListWidget::item:hover { background-color: #e8e8e8; }
        QListWidget::item:selected {
            background-color: #cce4f7;
            color: #1a1a1a;
        }
        QListWidget::item:focus { outline: none; border: none; }
    """
