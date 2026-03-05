from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import QSize


ICON_DIR  = "./icon"
IMAGE_DIR = "./img"
QT_DIR    = "./qt"


class Colors:

    BG_WINDOW     = "rgb(30,30,30)"
    BG_SIDEBAR    = "rgb(37,37,38)"
    BG_MAIN       = "rgb(62,62,66)"
    BG_PRIMARY    = "#1e1e1e"
    BG_SECONDARY  = "#252525"
    BG_ELEVATED   = "#2d2d2d"
    BG_HOVER      = "#3a3a3a"
    BG_SELECTED   = "#454545"

    BORDER        = "#333333"
    BORDER_LIGHT  = "#444444"
    BORDER_FOCUS  = "#555555"

    TEXT_PRIMARY   = "#e0e0e0"
    TEXT_SECONDARY = "#aaaaaa"
    TEXT_MUTED     = "#666666"
    TEXT_WHITE     = "#ffffff"
    TEXT_BRIGHT    = "#dddddd"

    ACCENT_BLUE        = "#3498db"
    ACCENT_BLUE_BG     = "#25394d"
    ACCENT_BLUE_BG_HVR = "#2c3e50"
    ACCENT_BLUE_TEXT   = "#5dade2"

    ACCENT_GREEN        = "#2ecc71"
    ACCENT_GREEN_DARK   = "#27ae60"
    ACCENT_GREEN_BG     = "#1e3a2a"
    ACCENT_GREEN_BG_HVR = "#225633"

    ACCENT_RED  = "#e74c3c"

    STATUS_PASS    = "#27ae60"
    STATUS_FAIL    = ACCENT_RED
    STATUS_PENDING = "#f39c12"


class Styles:

    BUTTON_ICON = f"""
        QPushButton {{
            background-color: transparent;
            border: none;
            padding: 5px;
        }}
        QPushButton:hover {{
            background-color: rgba(255, 255, 255, 0.1);
            border-radius: 4px;
        }}
        QPushButton:pressed {{
            background-color: rgba(255, 255, 255, 0.2);
        }}
    """

    BUTTON_DEFAULT = f"""
        QPushButton {{
            background-color: {Colors.BG_SELECTED};
            color: {Colors.TEXT_PRIMARY};
            outline: none;
        }}
        QPushButton:hover {{
            background-color: {Colors.BG_HOVER};
        }}
        QPushButton:pressed {{
            background-color: {Colors.BG_SECONDARY};
        }}
    """

    BUTTON_BLUE = f"""
        QPushButton {{
            background-color: {Colors.ACCENT_BLUE_BG};
            border: 1px solid {Colors.ACCENT_BLUE};
            border-radius: 4px;
            color: {Colors.ACCENT_BLUE_TEXT};
        }}
        QPushButton:hover {{ background-color: {Colors.ACCENT_BLUE_BG_HVR}; }}
        QPushButton:pressed {{ background-color: {Colors.BG_ELEVATED}; }}
    """

    BUTTON_GREEN = f"""
        QPushButton {{
            background-color: {Colors.ACCENT_GREEN_BG};
            border: 1px solid {Colors.ACCENT_GREEN};
            border-radius: 4px;
            color: {Colors.ACCENT_GREEN_DARK};
        }}
        QPushButton:hover {{ background-color: {Colors.ACCENT_GREEN_BG_HVR}; }}
        QPushButton:pressed {{ background-color: {Colors.BG_ELEVATED}; }}
    """

    LIST_WIDGET = f"""
        QListWidget {{
            border: 1px solid {Colors.BORDER};
            background-color: {Colors.BG_SECONDARY};
            alternate-background-color: #2a2a2a;
            outline: none;
        }}
        QListWidget::item {{
            border-bottom: 1px solid {Colors.BORDER};
            color: {Colors.TEXT_PRIMARY};
        }}
        QListWidget::item:hover {{
            background-color: {Colors.BG_ELEVATED};
        }}
        QListWidget::item:selected {{
            background-color: {Colors.BG_HOVER};
            color: {Colors.TEXT_WHITE};
        }}
        QListWidget::item:selected:hover {{
            background-color: {Colors.BG_SELECTED};
        }}
        QListWidget::item:focus {{
            outline: none;
            border: none;
        }}
    """

    SCROLLBAR_HIDDEN = """
        QScrollBar:vertical {
            background: transparent;
            width: 0px;
        }
        QScrollBar::handle:vertical { background: transparent; }
        QScrollBar::add-line:vertical,
        QScrollBar::sub-line:vertical { height: 0px; background: transparent; }
        QScrollBar::add-page:vertical,
        QScrollBar::sub-page:vertical { background: transparent; }
    """

    SCROLLBAR_THIN = f"""
        QScrollBar:vertical {{
            background: transparent;
            width: 8px;
            margin: 0px;
        }}
        QScrollBar::handle:vertical {{
            background-color: #c0c0c0;
            border-radius: 4px;
            min-height: 20px;
        }}
        QScrollBar::handle:vertical:hover {{ background-color: #a0a0a0; }}
        QScrollBar::add-line:vertical,
        QScrollBar::sub-line:vertical {{ height: 0px; border: none; background: transparent; }}
        QScrollBar::add-page:vertical,
        QScrollBar::sub-page:vertical {{ background: transparent; }}
    """

    GROUP_BOX = f"""
        QGroupBox {{
            border: 1px solid {Colors.BG_HOVER};
            border-radius: 4px;
            font-weight: bold;
            color: {Colors.TEXT_BRIGHT};
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            color: {Colors.TEXT_PRIMARY};
        }}
    """

    LINE_EDIT = f"""
        QLineEdit {{
            padding: 8px 15px;
            border: 1px solid #ddd;
            border-radius: 8px;
            font-size: 14px;
            background-color: white;
        }}
        QLineEdit:focus {{
            border: 2px solid {Colors.ACCENT_BLUE};
        }}
    """

    PROGRESS_BAR = f"""
        QProgressBar {{
            border: none;
            background-color: rgba(255, 255, 255, 0.1);
            max-height: 10px;
        }}
        QProgressBar::chunk {{
            background-color: {Colors.ACCENT_GREEN_DARK};
        }}
    """

    CARD_NAME_LABEL = f"""
        background-color: gray;
        border: none;
        padding: 5px;
        font-size: 16px;
        font-weight: bold;
        color: rgb(45,45,48);
    """

    CARD_IMAGE_LABEL = """
        background-color: rgb(0,122,204);
        font-size: 16px;
        font-weight: bold;
    """

    FILTER_FRAME = """
        QWidget#frame {
            background-color: rgb(230, 230, 230);
            border-radius: 15px;
            border: 2px solid rgb(100, 100, 100);
        }
        QLabel {
            background-color: transparent;
            font-weight: bold;
        }
    """

    FILTER_BUTTON = f"""
        QPushButton {{
            padding: 8px 15px;
            border: 1px solid #ddd;
            border-radius: 8px;
            background-color: white;
        }}
        QPushButton::menu-indicator {{ image: none; }}
        QPushButton:hover {{ background-color: rgba(100, 100, 100, 0.1); }}
        QPushButton:pressed {{ background-color: rgba(255, 255, 255, 0.2); }}
    """


class Icons:

    _cache: dict[str, QIcon] = {}

    @classmethod
    def get(cls, filename: str) -> QIcon:
        if filename not in cls._cache:
            cls._cache[filename] = QIcon(f"{ICON_DIR}/{filename}")
        return cls._cache[filename]

    @classmethod
    def MINIMIZE(cls) -> QIcon: return cls.get("minimize.png")
    @classmethod
    def MAXIMIZE(cls) -> QIcon: return cls.get("maximize.png")
    @classmethod
    def CLOSE(cls)    -> QIcon: return cls.get("close.png")

    @classmethod
    def MENU(cls)      -> QIcon: return cls.get("menu.png")
    @classmethod
    def SENSOR(cls)    -> QIcon: return cls.get("sensor.png")
    @classmethod
    def TEST_MENU(cls) -> QIcon: return cls.get("test_menu.png")

    @classmethod
    def RUN(cls)       -> QIcon: return cls.get("run.png")
    @classmethod
    def STOP(cls)      -> QIcon: return cls.get("stop.png")
    @classmethod
    def RUN_ALL(cls)   -> QIcon: return cls.get("run_all.png")
    @classmethod
    def EXPORT(cls)    -> QIcon: return cls.get("export.png")
    @classmethod
    def FILTER(cls)    -> QIcon: return cls.get("filter.png")
    @classmethod
    def CLOSE_BLK(cls) -> QIcon: return cls.get("close_black.png")
    @classmethod
    def CLEAR(cls)     -> QIcon: return cls.get("clear_logs.png")
    @classmethod
    def COPY(cls)      -> QIcon: return cls.get("copy.png")
    @classmethod
    def SAVE(cls)      -> QIcon: return cls.get("save.png")

    @classmethod
    def SUCCESS(cls) -> QIcon: return cls.get("success.png")
    @classmethod
    def FAIL(cls)    -> QIcon: return cls.get("fail.png")
    @classmethod
    def PENDING(cls) -> QIcon: return cls.get("pending.png")
    @classmethod
    def RUNNING(cls) -> QIcon: return cls.get("running.png")
    @classmethod
    def UNKNOWN(cls) -> QIcon: return cls.get("unknown_status.png")

    @classmethod
    def for_status(cls, status: str, is_running: bool = False) -> QIcon:
        if is_running:
            return cls.RUNNING()
        if status == "Passed":
            return cls.SUCCESS()
        if status == "Failed":
            return cls.FAIL()
        if status == "Pending":
            return cls.PENDING()
        return cls.UNKNOWN()


class Layout:
    ICON_SIZE_SM = QSize(16, 16)
    ICON_SIZE_MD = QSize(24, 24)
    ICON_SIZE_LG = QSize(32, 32)

    CARD_WIDTH   = 350
    CARD_HEIGHT  = 300
    CARD_H_GAP   = 10
    CARD_V_GAP   = 30

    TEST_ITEM_HEIGHT = 75
    TEST_LIST_MAX_H  = 400