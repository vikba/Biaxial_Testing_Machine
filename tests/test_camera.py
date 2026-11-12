"""Camera migration checks using real Qt/OpenCV and a simulated VmbPy API."""
from contextlib import contextmanager
from importlib.util import find_spec
from threading import Event, get_ident
from time import monotonic, sleep
from types import ModuleType, SimpleNamespace
import os
import sys
import unittest
from unittest.mock import MagicMock, Mock, mock_open, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
AVAILABLE = all(find_spec(name) is not None for name in ('PyQt6', 'numpy', 'cv2'))
if AVAILABLE:
    import cv2
    import numpy as np
    from PyQt6.QtCore import QPoint, QPointF
    from PyQt6.QtWidgets import QApplication
    from sources.camerawindow import VideoThread, VideoWindow


def pump(predicate, timeout=3):
    deadline = monotonic() + timeout
    while not predicate() and monotonic() < deadline:
        QApplication.processEvents()
        sleep(.001)
    QApplication.processEvents()
    if not predicate():
        raise AssertionError('Camera operation did not complete')


@unittest.skipUnless(AVAILABLE, 'Install Qt/OpenCV dependencies for camera tests')
class CameraTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.sdk = ModuleType('vmbpy')
        self.sdk.VmbTimeout = type('VmbTimeout', (Exception,), {})
        self.sdk.VmbFeatureError = type('VmbFeatureError', (Exception,), {})
        self.mono, self.bgr = Mock(name='Mono8'), Mock(name='Bgr8')
        self.mono.get_convertible_formats.return_value = ()
        self.bgr.get_convertible_formats.return_value = ()
        self.sdk.PixelFormat = SimpleNamespace(Mono8=self.mono, Bgr8=self.bgr)
        self.sdk.FrameStatus = SimpleNamespace(Complete=0)
        self.module_patch = patch.dict(sys.modules, {'vmbpy': self.sdk})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.thread = VideoThread()
        self.errors = []
        self.thread.signal_error.connect(self.errors.append)
        self.thread._draw_coordinate_system = Mock()
        self.thread._detectMarkers = Mock(side_effect=lambda image, **kwargs: (image.copy(), image.copy(), []))
        self.camera = MagicMock()
        self.camera.get_id.return_value = 'DEV_TEST'
        self.camera.__enter__.return_value = self.camera
        self.camera.get_pixel_formats.return_value = (self.mono,)
        self.thread._cam = self.camera
        self.frames = []
        self.thread.signal_change_pixmap.connect(self.frames.append)
        self.release = Event()

    def tearDown(self):
        self.release.set()
        self.thread.stop()
        pump(lambda: not self.thread.isRunning())

    def frame(self, image, pixel_format=None, status=0):
        frame = Mock()
        frame.get_status.return_value = status
        frame.get_pixel_format.return_value = pixel_format or self.mono
        frame.as_opencv_image.return_value = image
        @contextmanager
        def acquired(**kwargs):
            yield frame
            image.fill(0)  # Simulate an SDK-owned buffer being reused after context exit.
        self.camera.get_frame_with_context.side_effect = acquired
        return frame

    def system(self):
        vmb = MagicMock()
        vmb.__enter__.return_value = vmb
        vmb.get_all_cameras.return_value = (self.camera,)
        self.sdk.VmbSystem = SimpleNamespace(get_instance=lambda: vmb)
        self.camera.get_feature_by_name.side_effect = self.sdk.VmbFeatureError('unsupported')
        return vmb

    def test_camera_identity_and_ambiguous_discovery(self):
        other = Mock()
        other.get_id.return_value = 'OTHER'
        with self.assertRaisesRegex(RuntimeError, 'runtime.camera_id'):
            self.thread._select_camera((self.camera, other))
        self.assertIs(self.thread._select_camera((self.camera,)), self.camera)
        with self.assertRaisesRegex(RuntimeError, 'DEV_TEST'):
            self.thread._select_camera((other,))
        self.assertIs(self.thread._select_camera((other, self.camera)), self.camera)

    def test_native_dimensions_and_owned_frame_copy(self):
        self.frame(np.full((120, 200, 1), 128, dtype=np.uint8))
        self.assertTrue(self.thread._grab_frame())
        self.assertEqual(self.thread._frame_shape, (120, 200))
        self.assertEqual(self.thread._img_track.shape, (120, 200))
        np.testing.assert_array_equal(self.frames[-1], np.full((120, 200), 128, dtype=np.uint8))
        self.camera.get_frame_with_context.assert_called_once_with(timeout_ms=500)

    def test_color_and_high_bit_depth_conversion(self):
        color = np.full((120, 200, 3), 255, dtype=np.uint8)
        self.frame(color, self.bgr)
        self.thread._grab_frame()
        self.assertEqual(self.frames[-1].shape, (120, 200))
        self.assertTrue(np.all(self.frames[-1] == 255))
        high_bits = Mock()
        high_bits.get_convertible_formats.return_value = (self.mono,)
        frame = self.frame(np.zeros((120, 200, 1), dtype=np.uint16), high_bits)
        frame.convert_pixel_format.return_value.as_opencv_image.return_value = np.full((120, 200, 1), 77, dtype=np.uint8)
        self.thread._grab_frame()
        frame.convert_pixel_format.assert_called_once_with(self.mono)
        self.assertTrue(np.all(self.frames[-1] == 77))

    def test_incomplete_and_unsupported_frames_are_rejected(self):
        self.frame(np.zeros((120, 200), dtype=np.uint8), status=-1)
        with self.assertRaisesRegex(RuntimeError, 'Incomplete'):
            self.thread._grab_frame()
        unsupported = Mock()
        unsupported.get_convertible_formats.return_value = ()
        self.frame(np.zeros((120, 200), dtype=np.uint8), unsupported)
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            self.thread._grab_frame()
        self.thread._detectMarkers.assert_not_called()

    def test_timeout_uses_sdk_type_and_invalidates_active_tracking(self):
        self.camera.get_frame_with_context.side_effect = self.sdk.VmbTimeout('no frame')
        self.assertFalse(self.thread._grab_frame())
        self.thread._track_marks = True
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.thread._grab_frame()
        self.camera.get_frame_with_context.side_effect = ValueError('unrelated timeout text')
        with self.assertRaises(ValueError):
            self.thread._grab_frame()

    def test_resolution_change_discards_marker_references(self):
        self.frame(np.full((120, 200), 128, dtype=np.uint8))
        self.thread._grab_frame()
        self.thread._point1.append((10, 10))
        self.thread._track_marks = True
        self.frame(np.full((180, 320), 128, dtype=np.uint8))
        with self.assertRaisesRegex(RuntimeError, 'resolution changed'):
            self.thread._grab_frame()
        self.assertFalse(any(self.thread._marks_groups))
        self.assertFalse(self.thread._track_marks)
        self.assertEqual(self.thread._frame_shape, (180, 320))

    def test_native_roi_clamping_and_marker_detection(self):
        self.thread._detectMarkers = VideoThread._detectMarkers.__get__(self.thread)
        image = np.full((120, 200), 255, dtype=np.uint8)
        cv2.circle(image, (30, 30), 12, 0, -1)
        cv2.circle(image, (150, 90), 12, 0, -1)
        self.thread._frame_shape = image.shape
        self.thread.update_roi(-50, -50, 80, 80)
        self.thread._apply_bg_correction = False
        self.frame(image.copy())
        self.thread._grab_frame()
        self.assertEqual((self.thread._roi_x1, self.thread._roi_y1, self.thread._roi_x2, self.thread._roi_y2), (0, 0, 80, 80))
        _, _, points = self.thread._detectMarkers(image, apply_bg_correction=False)
        self.assertEqual(len(points), 1)
        self.assertLess(abs(points[0][0] - 30), 2)

    def test_marker_initialization_requires_distinct_detections(self):
        self.frame(np.zeros((120, 200), dtype=np.uint8))
        self.thread._grab_frame()
        self.thread._detectMarkers.side_effect = lambda image, **kwargs: (image, image, [(30, 30)])
        recorded = []
        self.thread.signal_markers_recorded.connect(recorded.append)
        self.thread.set_init_points([(29, 29), (29, 31), (31, 29), (31, 31)])
        self.thread._grab_frame()
        self.assertFalse(recorded)
        self.assertTrue(any('distinct markers' in error for error in self.errors))

    def test_settings_are_clamped_and_written_only_in_camera_thread(self):
        self.system()
        writes = []
        features = {}
        for name, bounds, step in (('Gain', (0.0, 12.0), .5), ('ExposureTime', (10.0, 1e6), 10.0)):
            feature = Mock()
            feature.is_readable.return_value = feature.is_writeable.return_value = True
            feature.get_range.return_value = bounds
            feature.get_increment.return_value = step
            feature.get.return_value = bounds[0]
            def write(value, feature=feature):
                writes.append((get_ident(), value))
                feature.get.return_value = value
            feature.set.side_effect = write
            features[name] = feature
        def lookup(name):
            if name not in features:
                raise self.sdk.VmbFeatureError('unsupported')
            return features[name]
        self.camera.get_feature_by_name.side_effect = lookup
        self.frame(np.full((120, 200), 128, dtype=np.uint8))
        self.thread.start()
        pump(lambda: bool(self.frames))
        self.thread.set_gain(7.3)
        self.thread.set_exposure(1e6)
        pump(lambda: features['ExposureTime'].get.return_value == 400000)
        self.assertEqual(features['Gain'].get.return_value, 7.0)
        self.assertTrue(all(thread_id != get_ident() for thread_id, _ in writes))
        self.assertEqual(self.thread._camera_settings['Gain'][:3], (0, 12, .5))

    def test_stop_returns_without_waiting_for_frame(self):
        self.system()
        entered = Event()
        @contextmanager
        def stalled(**kwargs):
            entered.set()
            self.release.wait(.5)
            raise self.sdk.VmbTimeout('no frame')
            yield  # pragma: no cover
        self.camera.get_frame_with_context.side_effect = stalled
        self.thread.start()
        pump(entered.is_set)
        started = monotonic()
        self.thread.stop()
        self.assertLess(monotonic() - started, .1)
        self.release.set()
        pump(lambda: not self.thread.isRunning())
        self.assertFalse(self.frames)
        self.camera.__exit__.assert_called_once()

    def test_missing_sdk_does_not_prevent_application_import(self):
        with patch.dict(sys.modules, {'vmbpy': None}):
            self.thread.run()
            self.assertTrue(any('matching VmbPy' in message for message in self.errors))
            if all(find_spec(name) is not None for name in ('pyqtgraph', 'openpyxl', 'serial', 'zaber_motion')):
                from sources.biaxmainwindow import BiaxMainWindow
                window = BiaxMainWindow()
                window.close()

    def test_letterboxed_clicks_map_to_native_coordinates(self):
        window = VideoWindow(self.thread)
        window.show()
        QApplication.processEvents()
        window.update_image(np.zeros((200, 400), dtype=np.uint8))
        window.start_point_collection()
        # Image occupies y=80..400 inside its 640x480 label.
        for point in (QPoint(320, 10), QPoint(320, 240)):
            local = window._image_label.mapTo(window, point)
            window.mouseReleaseEvent(SimpleNamespace(position=lambda local=local: QPointF(local)))
        self.assertEqual(window.clicked_points, [(200.0, 100.0)])
        window._update_camera_controls({})
        self.assertFalse(window.point_collection_mode)
        self.assertFalse(window.init_button.isEnabled())
        window.close()

    def test_reconnection_clears_tracking_and_keeps_camera_identity(self):
        system = self.system()
        other = MagicMock()
        other.get_id.return_value = 'OTHER'
        system.get_all_cameras.side_effect = [(self.camera,), (other,), (self.camera,)]
        self.thread.FRAME_GRAB_DELAY_MS = self.thread.CAMERA_RECONNECT_DELAY_MS = 5
        frame = self.frame(np.full((120, 200), 128, dtype=np.uint8))
        entered = Event()
        count = 0
        @contextmanager
        def acquire(**kwargs):
            nonlocal count
            count += 1
            if count == 2:
                entered.set()
                self.release.wait(1)
                raise self.sdk.VmbTimeout('unplugged')
            yield frame
        self.camera.get_frame_with_context.side_effect = acquire
        self.thread.start()
        pump(entered.is_set)
        self.thread._point1.append((30, 30))
        self.thread._track_marks = True
        self.release.set()
        pump(lambda: self.camera.__enter__.call_count == 2 and len(self.frames) > 1)
        self.assertEqual(self.thread._camera_id, 'DEV_TEST')
        self.assertFalse(any(self.thread._marks_groups))
        self.assertFalse(self.thread._track_marks)
        other.__enter__.assert_not_called()
        self.assertTrue(any('DEV_TEST' in error for error in self.errors))

    def test_image_save_failures_are_reported(self):
        self.thread.save_image('unused.png')
        self.thread._img = np.zeros((10, 10), dtype=np.uint8)
        with patch('sources.camerawindow.cv2.imwrite', return_value=False):
            self.thread.save_image('unused.png')
        with patch('sources.camerawindow.cv2.imwrite', side_effect=cv2.error('writer unavailable')):
            self.thread.save_image('unused.png')
        self.assertEqual(len(self.errors), 3)
        self.assertTrue(all('Could not save camera image' in error for error in self.errors))

    def test_camera_fault_fails_only_a_video_dependent_test(self):
        if not all(find_spec(name) is not None for name in ('pyqtgraph', 'openpyxl', 'serial', 'zaber_motion')):
            self.skipTest('Full application dependencies are required')
        from sources.biaxmainwindow import BiaxMainWindow
        test = Mock()
        test._use_video = True
        test.isRunning.return_value = True
        window = SimpleNamespace(_mec_test=test, _show_warning=Mock())
        BiaxMainWindow._handle_camera_error(window, 'connection lost')
        self.assertIn('connection lost', str(test._fail.call_args.args[0]))
        window._show_warning.assert_not_called()
        test._fail.reset_mock()
        test._use_video = False
        BiaxMainWindow._handle_camera_error(window, 'connection lost')
        test._fail.assert_not_called()
        window._show_warning.assert_called_once()

    def test_camera_window_stays_alive_until_worker_exits(self):
        self.system()
        entered = Event()
        @contextmanager
        def stalled(**kwargs):
            entered.set()
            self.release.wait(.5)
            raise self.sdk.VmbTimeout('no frame')
            yield  # pragma: no cover
        self.camera.get_frame_with_context.side_effect = stalled
        window = VideoWindow(self.thread)
        window.show()
        self.thread.start()
        pump(entered.is_set)
        window.close()
        self.assertTrue(window.isVisible())
        self.release.set()
        pump(lambda: not self.thread.isRunning() and not window.isVisible())

    def test_camera_id_config_roundtrip_and_legacy_default(self):
        from sources.configuration import ConfigRepository
        repository = ConfigRepository('unused.json')
        with patch('builtins.open', mock_open(read_data='{"runtime":{"camera_id":" DEV_TEST "}}')), \
                patch('os.path.exists', return_value=True):
            config = repository.load()
        self.assertEqual(config.runtime.camera_id, 'DEV_TEST')
        self.assertEqual(config.to_dict()['runtime']['camera_id'], 'DEV_TEST')
        with patch('builtins.open', mock_open(read_data='{"unit":"gram"}')), \
                patch('os.path.exists', return_value=True):
            self.assertEqual(repository.load().runtime.camera_id, '')


if __name__ == '__main__':
    unittest.main()
