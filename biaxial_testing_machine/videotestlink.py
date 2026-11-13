"""One detachable GUI-thread relay between camera and test mailboxes."""
from PyQt6.QtCore import QObject, pyqtSlot


class VideoTestLink(QObject):
    def __init__(self, video, test, parent):
        super().__init__(parent)
        self.video, self.test, self.active = video, test, True
        self.connections = (
            (video.signal_markers_recorded, self.initialize),
            (video.signal_markers_coordinates, self.update),
            (test.signal_start_stop_tracking, self.track),
            (test.signal_save_image, self.save),
        )
        for signal, slot in self.connections:
            signal.connect(slot)

    @pyqtSlot(list)
    def initialize(self, points):
        if self.active:
            self.test.init_markers(points)

    @pyqtSlot(list)
    def update(self, points):
        if self.active:
            self.test.update_markers(points)

    @pyqtSlot(bool)
    def track(self, enabled):
        if self.active:
            self.video.start_stop_tracking(enabled)

    @pyqtSlot(str)
    def save(self, path):
        if self.active:
            self.video.save_image(path)

    def detach(self):
        self.active = False  # Also rejects deliveries queued before disconnect.
        for signal, slot in self.connections:
            try:
                signal.disconnect(slot)
            except (TypeError, RuntimeError):
                pass
        self.video.start_stop_tracking(False)
        self.deleteLater()
