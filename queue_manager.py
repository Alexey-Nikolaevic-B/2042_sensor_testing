from PyQt5.QtCore import QObject, pyqtSignal


class QueueManager(QObject):

    queue_changed = pyqtSignal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._queue: list = []

    @property
    def queue(self) -> list:
        return list(self._queue)

    def add_to_queue(self, test_widget) -> None:
        if test_widget not in self._queue:
            self._queue.append(test_widget)
            self.queue_changed.emit(self.queue)
            self._process_queue()

    def remove_from_queue(self, test_widget) -> None:
        if test_widget in self._queue:
            self._queue.remove(test_widget)
            self.queue_changed.emit(self.queue)

    def clear(self) -> None:
        self._queue.clear()
        self.queue_changed.emit(self.queue)

    def _process_queue(self) -> None:
        pass