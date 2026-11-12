"""Fault injection with real Qt threads and simulated hardware (no vendor calls)."""
from importlib.util import find_spec
from threading import Event, Thread
from time import monotonic, sleep
import unittest
from unittest.mock import Mock, patch

from sources.configuration import MachineConfig


class ValidationTests(unittest.TestCase):
    def test_invalid_parameters(self):
        for tab, section, field in (
            (0, 'sample', 'thickness'), (2, 'sample', 'length1'),
            (0, 'load_test', 'halfcycltime'), (0, 'load_test', 'disp_guess1'),
            (2, 'displacement_test', 'disptestspeed1'),
            (2, 'displacement_test', 'disptestlen2'),
        ):
            for value in ('bad', '0', '-1', 'nan', 'inf'):
                config = MachineConfig()
                setattr(getattr(config, section), field, value)
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    config.validate(tab)

    def test_cycle_counts_and_loads(self):
        for tab, field, value in ((0, 'cycleprecond', '-1'), (0, 'cycletest', '1.5'),
                                  (2, 'disptestcycle', '0'), (2, 'disptestcycle', 'nan')):
            config = MachineConfig()
            setattr(config.load_test if tab == 0 else config.displacement_test, field, value)
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                config.validate(tab)
        config = MachineConfig()
        config.load_test.cycleprecond = '0'
        config.validate(0)
        config.load_test.cycletest = '0'
        with self.assertRaises(ValueError):
            config.validate(0)
        config = MachineConfig()
        config.load_test.tareload1 = config.load_test.load_test1
        with self.assertRaisesRegex(ValueError, 'Target loads'):
            config.validate(0)
        config.load_test.tareload1 = '0'
        config.validate(0)
        config.displacement_test.disptestlen1 = '167'
        with self.assertRaisesRegex(ValueError, 'travel'):
            config.validate(2)


RUNTIME_AVAILABLE = all(find_spec(name) is not None for name in ('PyQt6', 'numpy', 'openpyxl', 'serial', 'zaber_motion'))
if RUNTIME_AVAILABLE:
    from PyQt6.QtCore import QCoreApplication, QTimer
    from sources.hardware import HardwareWorker, MotorDAQInterface
    from sources.sessioncontroller import TestSessionController
    from sources.sessionstate import SessionState
    from sources.unit import Unit


def pump(predicate, timeout=3):
    deadline = monotonic() + timeout
    while not predicate() and monotonic() < deadline:
        QCoreApplication.processEvents()
        sleep(.001)
    QCoreApplication.processEvents()
    if not predicate():
        raise AssertionError('Qt operation did not complete')


@unittest.skipUnless(RUNTIME_AVAILABLE, 'Install Qt and runtime dependencies for lifecycle tests')
class LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.axis1, self.axis2 = Mock(), Mock()
        self.axis1.get_position.return_value = self.axis2.get_position.return_value = 40.0
        self.acquisition = Mock()
        self.acquisition.read.return_value = (1.0, 1.0)
        self.acquisition.zero.return_value = (0.0, 0.0)
        self.releases = []
        self.tests = []
        self.interfaces = []
        axes, acquisition = (self.axis1, self.axis2), self.acquisition
        def motors(worker):
            worker._axis1, worker._axis2 = axes
            worker._connection_z = Mock()
        self.motor_patch = patch.object(HardwareWorker, '_connect_motors', motors)
        self.daq_patch = patch.object(HardwareWorker, '_connect_daq', lambda worker: setattr(worker, '_acquisition', acquisition))
        self.motor_patch.start()
        self.daq_patch.start()
        self.addCleanup(self.motor_patch.stop)
        self.addCleanup(self.daq_patch.stop)

    def tearDown(self):
        for release in self.releases:
            release.set()
        for test in self.tests:
            test.stop()
        for interface in self.interfaces:
            interface.stop()
        pump(lambda: all(not item._thread.isRunning() for item in self.interfaces)
             and all(not test.isRunning() for test in self.tests))

    def connect(self):
        interface = MotorDAQInterface(Unit.Newton)
        self.interfaces.append(interface)
        pump(lambda: interface._worker._connected)
        return interface

    def session(self):
        controller = TestSessionController()
        config = MachineConfig()
        config.runtime.sample_name = 'sample'
        controller.connect_hardware(config)
        interface = controller.hardware
        self.interfaces.append(interface)
        pump(lambda: interface._worker._connected)
        interface._worker._force_zeroed = interface._worker._position_zeroed = True
        return controller, config

    def test_connection_keeps_gui_responsive(self):
        entered, release, heartbeat = Event(), Event(), Event()
        self.releases.append(release)
        def connect(worker):
            entered.set()
            release.wait(2)
            worker._axis1, worker._axis2 = self.axis1, self.axis2
        with patch.object(HardwareWorker, '_connect_motors', connect):
            interface = MotorDAQInterface(Unit.Newton)
            self.interfaces.append(interface)
            QTimer.singleShot(5, heartbeat.set)
            pump(lambda: entered.is_set() and heartbeat.is_set())
            self.assertFalse(interface._worker._connected)
            release.set()
            pump(lambda: interface._worker._connected)

    def test_timeout_rejects_late_motion_and_queued_commands(self):
        interface = self.connect()
        interface.COMMAND_TIMEOUT = .05
        entered, release = Event(), Event()
        self.releases.append(release)
        original = HardwareWorker.move_velocity_ax1
        def delayed(worker, speed):
            entered.set()
            release.wait(2)
            original(worker, speed)
        errors = []
        interface.signal_error.connect(errors.append)
        with patch.object(HardwareWorker, 'move_velocity_ax1', delayed):
            interface.move_velocity_ax1(.1)
            pump(entered.is_set)
            interface.move_velocity_ax2(.1)
            pump(lambda: interface._worker._motion_blocked.is_set())
            self.assertTrue(any('unconfirmed' in error for error in errors))
            release.set()
            pump(lambda: not interface._thread.isRunning())
        self.axis1.move_velocity.assert_not_called()
        self.axis2.move_velocity.assert_not_called()
        self.axis1.stop.assert_called()
        self.axis2.stop.assert_called()

    def test_test_worker_wait_is_bounded(self):
        interface = self.connect()
        interface.COMMAND_TIMEOUT = .05
        release = Event()
        self.releases.append(release)
        errors = []
        def command():
            try:
                interface._invoke('get_positions', wait=True)
            except Exception as error:
                errors.append(error)
        with patch.object(HardwareWorker, 'get_positions', lambda worker: release.wait(2)):
            caller = Thread(target=command)
            caller.start()
            pump(lambda: not caller.is_alive())
            caller.join()
            self.assertIsInstance(errors[0], TimeoutError)
            self.assertTrue(interface._worker._motion_blocked.is_set())
            release.set()
            pump(lambda: bool(interface._worker._force_fault))

    def test_stop_cancels_queued_motion_without_faulting(self):
        interface = self.connect()
        entered, release = Event(), Event()
        self.releases.append(release)
        def delayed(worker):
            entered.set()
            release.wait(2)
        with patch.object(HardwareWorker, 'get_positions', delayed):
            interface._invoke('get_positions', wait=False)
            pump(entered.is_set)
            interface.move_velocity_ax2(.1)
            interface.stop_test()
            release.set()
            pump(lambda: not interface._pending)
        self.axis2.move_velocity.assert_not_called()
        self.assertIsNone(interface._worker._force_fault)

    def test_disconnect_attempts_both_stops_and_closes_connections(self):
        interface = self.connect()
        connection = interface._worker._connection_z
        self.axis1.stop.side_effect = RuntimeError('axis one offline')
        interface.stop()
        pump(lambda: not interface._thread.isRunning())
        self.axis2.stop.assert_called()
        self.acquisition.close.assert_called_once()
        connection.close.assert_called_once()

    def test_invalid_start_issues_no_commands(self):
        controller, config = self.session()
        config.load_test.halfcycltime = '0'
        with self.assertRaises(ValueError):
            controller.start_test(config, 0)
        self.assertIsNone(controller.active_test)
        self.axis1.get_position.assert_not_called()
        self.axis1.move_velocity.assert_not_called()
        self.axis1.stop.assert_not_called()

    def test_startup_failure_restores_error_state(self):
        controller, config = self.session()
        errors = []
        controller.signal_error.connect(errors.append)
        with patch.object(HardwareWorker, 'zeroPosition', side_effect=OSError('position offline')):
            test = controller.start_test(config, 0)
            self.tests.append(test)
            pump(lambda: controller.active_test is None and controller.state == SessionState.ERROR)
        self.assertTrue(any('position offline' in message for message in errors))
        self.axis1.stop.assert_called()
        self.axis2.stop.assert_called()
        self.axis1.move_velocity.assert_not_called()

    def test_timer_failures_stop_both_test_types(self):
        from sources.loadtest import LoadControlTest
        from sources.displacementtest import DisplacementControlTest
        for tab, test_type in ((0, LoadControlTest), (2, DisplacementControlTest)):
            with self.subTest(tab=tab):
                controller, config = self.session()
                errors, saved = [], []
                controller.signal_error.connect(errors.append)
                controller.signal_save_charts.connect(lambda: saved.append(True))
                with patch.object(test_type, '_sample_time', 5), \
                        patch.object(test_type, '_record_sample', side_effect=OSError('calculation failed')):
                    test = controller.start_test(config, tab)
                    self.tests.append(test)
                    pump(lambda: controller.active_test is None and controller.state == SessionState.ERROR)
                self.assertIn('calculation failed', test._failure)
                self.assertFalse(saved)
                self.axis1.stop.assert_called()
                self.axis2.stop.assert_called()
                self.interfaces[-1].stop()
                pump(lambda: not self.interfaces[-1]._thread.isRunning())

    def test_stop_during_startup_prevents_motion(self):
        controller, config = self.session()
        entered, release = Event(), Event()
        self.releases.append(release)
        def delayed(worker):
            entered.set()
            release.wait(2)
            return (40.0, 40.0)
        with patch.object(HardwareWorker, 'zeroPosition', delayed):
            test = controller.start_test(config, 0)
            self.tests.append(test)
            pump(entered.is_set)
            controller.stop_test()
            release.set()
            pump(lambda: controller.active_test is None and controller.state == SessionState.READY)
        self.assertIsNone(test._failure)
        self.axis1.move_velocity.assert_not_called()
        self.axis2.move_velocity.assert_not_called()

    def test_cycle_completion_and_export_failure(self):
        from sources.loadtest import LoadControlTest
        from sources.displacementtest import DisplacementControlTest
        self.acquisition.read.return_value = (1e6, 1e6)
        for axis in (self.axis1, self.axis2):
            axis.move_absolute.side_effect = lambda target, *args, axis=axis, **kwargs: setattr(axis.get_position, 'return_value', target)
        for tab, test_type in ((0, LoadControlTest), (2, DisplacementControlTest)):
            for error in (None, OSError('workbook locked')):
                with self.subTest(tab=tab, error=error):
                    controller, config = self.session()
                    config.load_test.cycleprecond = '0'
                    config.load_test.cycletest = config.displacement_test.disptestcycle = '1'
                    saved = []
                    controller.signal_save_charts.connect(lambda: saved.append(True))
                    with patch.object(test_type, '_sample_time', 5), \
                            patch.object(test_type, '_writeDataToFile', side_effect=error) as write:
                        test = controller.start_test(config, tab)
                        self.tests.append(test)
                        expected = SessionState.ERROR if error else SessionState.READY
                        pump(lambda: controller.active_test is None and controller.state == expected)
                    write.assert_called_once()
                    self.assertEqual(bool(saved), error is None)
                    self.assertEqual(test._failure, str(error) if error else None)
                    self.interfaces[-1].stop()
                    pump(lambda: not self.interfaces[-1]._thread.isRunning())


if __name__ == '__main__':
    unittest.main()
