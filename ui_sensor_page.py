from datetime import datetime

from PyQt5.QtWidgets import QWidget, QMenu
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5 import uic

from sensor_repository import SensorRepository
from ui_sensor_card import SensorCard
from ui_sensor_details import SensorDetails
from theme import Colors, Styles, Icons, Layout, QT_DIR


class SensorPage(QWidget):

    signal_closed_details = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self._repo   = SensorRepository.instance()
        self._cards: dict[str, SensorCard] = {}
        self._filtered: list[dict] = []
        self._active_filter: str | None = None
        self._selected_id:   str | None = None

        self.details_widget = SensorDetails()

        self._init_ui()
        self._init_state()
        self._setup_styles()
        self._connect_signals()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/sensor_page.ui", self)

        self._filter_menu = QMenu()
        self._rebuild_filter_menu()

        self.verticalLayout_2.addWidget(self.details_widget)
        self.details_widget.hide()

        self.no_results_label.hide()
        self.frame.hide()

    def _rebuild_filter_menu(self):
        self._filter_menu.clear()
        types = self._repo.get_types()
        for sensor_type in types:
            self._filter_menu.addAction(sensor_type)

        self.verticalLayout_2.addWidget(self.details_widget)
        self.details_widget.hide()

        self.no_results_label.hide()
        self.frame.hide()

    def _init_state(self):
        for sensor_data in self._repo.all_sensors():
            self._create_card(sensor_data)
        self._apply_filters()

    def _setup_styles(self):
        self.scroll_area.setStyleSheet(
            "QScrollArea { border: none; background-color: transparent; }"
            + Styles.SCROLLBAR_THIN
        )
        self.search_input.setStyleSheet(Styles.LINE_EDIT)
        self.frame.setStyleSheet(Styles.FILTER_FRAME)
        self.btn_filter.setStyleSheet(Styles.FILTER_BUTTON)
        self.btn_filter.setIcon(Icons.FILTER())
        self.btn_filter.setIconSize(Layout.ICON_SIZE_MD)
        self.btn_disable_filter.setIcon(Icons.CLOSE_BLK())
        self.btn_disable_filter.setIconSize(Layout.ICON_SIZE_MD)
        self.btn_disable_filter.setStyleSheet(Styles.BUTTON_ICON)
        self.results_label.setStyleSheet(
            f"color: {Colors.TEXT_MUTED}; font-size: 12px; font-weight: bold;"
        )
        self.no_results_label.setStyleSheet(
            f"font-size: 18px; color: {Colors.TEXT_SECONDARY}; "
            f"padding: 40px; text-align: center; font-weight: bold;"
        )

    def _connect_signals(self):
        self.search_input.textChanged.connect(self._apply_filters)
        self.btn_filter.clicked.connect(self._show_filter_menu)
        self.btn_disable_filter.clicked.connect(self._remove_filter)
        self.details_widget.signal_closed_details.connect(self._on_details_closed)
        self._filter_menu.triggered.connect(self._on_filter_selected)

        self._repo.sensor_added.connect(self._on_sensor_added)
        self._repo.sensor_updated.connect(self._on_sensor_updated)
        self._repo.sensor_deleted.connect(self._on_sensor_deleted)
        self._repo.sensors_loaded.connect(self._on_sensors_loaded)
        self._repo.test_updated.connect(self._on_test_updated)

    def _on_sensors_loaded(self):
        self._rebuild_filter_menu()
        for sensor_id in list(self._cards):
            if self._repo.get_sensor(sensor_id) is None:
                self._cards.pop(sensor_id).deleteLater()
        for sensor_data in self._repo.all_sensors():
            if sensor_data["id"] not in self._cards:
                self._create_card(sensor_data)
        self._apply_filters()

    def _on_sensor_added(self, sensor_data: dict):
        self._create_card(sensor_data)
        self._apply_filters()

    def _on_sensor_updated(self, sensor_data: dict):
        card = self._cards.get(sensor_data["id"])
        if card:
            card.refresh(sensor_data)
        if self._selected_id == sensor_data["id"]:
            self.details_widget.load_sensor(sensor_data)
        self._apply_filters()

    def _on_sensor_deleted(self, sensor_id: str):
        card = self._cards.pop(sensor_id, None)
        if card:
            card.deleteLater()
        if self._selected_id == sensor_id:
            self._close_details()
        self._apply_filters()

    def _on_test_updated(self, sensor_id: str, _test: dict):
        if self._selected_id == sensor_id:
            fresh = self._repo.get_sensor(sensor_id)
            if fresh:
                self.details_widget.load_sensor(fresh)

    def _show_filter_menu(self):
        pos = self.btn_filter.mapToGlobal(
            self.btn_filter.rect().bottomLeft()
        )
        self._filter_menu.exec_(pos)

    def _on_filter_selected(self, action):
        self._active_filter = action.text()
        self.lbl_selected_filter.setText(self._active_filter)
        self.frame.show()
        self.btn_filter.hide()
        self._apply_filters()

    def _remove_filter(self):
        self._active_filter = None
        self.frame.hide()
        self.btn_filter.show()
        self._apply_filters()

    def _apply_filters(self):
        sensors = self._repo.all_sensors()
        if self._active_filter:
            sensors = [
                s for s in sensors
                if s.get("type", "").lower() == self._active_filter.lower()
            ]
        query = self.search_input.text().strip().lower()
        if query:
            sensors = [
                s for s in sensors
                if query in s.get("name", "").lower()
            ]
        sensors.sort(
            key=lambda s: s.get("last_update", datetime.min), reverse=True
        )
        self._filtered = sensors
        self._rebuild_grid()

    def _on_card_clicked(self, sensor_id: str):
        sensor_data = self._repo.get_sensor(sensor_id)
        if sensor_data is None:
            return
        self._selected_id = sensor_id
        self.details_widget.load_sensor(sensor_data)
        self.details_widget.show()
        self._rebuild_grid()

    def _on_details_closed(self):
        self._close_details()

    def _close_details(self):
        self.details_widget.hide()
        self._selected_id = None
        self._rebuild_grid()

    def _create_card(self, sensor_data: dict):
        sensor_id = sensor_data["id"]
        if sensor_id in self._cards:
            return
        card = SensorCard(sensor_data, parent=self)
        card.clicked.connect(self._on_card_clicked)
        self._cards[sensor_id] = card

    def _rebuild_grid(self):
        for card in self._cards.values():
            card.hide()
        for i in reversed(range(self.cards_layout.count())):
            item = self.cards_layout.itemAt(i)
            if item.widget():
                self.cards_layout.removeWidget(item.widget())
            else:
                self.cards_layout.removeItem(item)
        for i in range(self.cards_layout.rowCount()):
            self.cards_layout.setRowStretch(i, 0)
        for i in range(self.cards_layout.columnCount()):
            self.cards_layout.setColumnStretch(i, 0)
        self.cards_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        if not self._filtered:
            self.no_results_label.show()
            self.results_label.setText("0 sensors found")
            self.cards_container.setFixedSize(0, 0)
            return

        self.no_results_label.hide()
        self.scroll_area.show()

        details_visible = self.details_widget.isVisible()
        available_width = (
            self.width() - self.details_widget.width() - 40
            if details_visible
            else self.width() - 40
        )

        cw   = Layout.CARD_WIDTH
        hgap = Layout.CARD_H_GAP
        vgap = Layout.CARD_V_GAP
        cols = max(1, available_width // (cw + hgap))
        rows = (len(self._filtered) + cols - 1) // cols

        self.cards_container.setFixedSize(
            cols * (cw + hgap) + 10,
            rows * (Layout.CARD_HEIGHT + vgap) + 20,
        )
        self.cards_layout.setHorizontalSpacing(hgap)
        self.cards_layout.setVerticalSpacing(vgap)
        self.cards_layout.setContentsMargins(0, 0, 0, 0)

        for i, sensor_data in enumerate(self._filtered):
            card = self._cards.get(sensor_data["id"])
            if card:
                card.show()
                card.setFixedSize(cw, Layout.CARD_HEIGHT)
                self.cards_layout.addWidget(
                    card, i // cols, i % cols, Qt.AlignTop | Qt.AlignLeft
                )

        self.cards_layout.setColumnStretch(cols, 1)
        self.cards_layout.setRowStretch(rows, 1)
        self.results_label.setText(
            f"Showing {len(self._filtered)} of {self._repo.count()} sensors"
        )
        self.cards_container.updateGeometry()
        self.scroll_area.viewport().update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rebuild_grid()