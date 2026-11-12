# Application structure

`biaxial_testing_machine.__main__.main()` creates the Qt main window.
`main.py` delegates to it for source-checkout compatibility. `TestSessionController` owns the hardware
interface and active test; load and displacement tests share `MechanicalTest`
for recording, export, and stop behavior.

`MotorDAQInterface` dispatches motor commands to `HardwareWorker` in a Qt thread.
GUI commands return immediately and complete through `signal_command_finished`;
commands from test threads wait at most five seconds. Queued commands carry a
deadline and a cancellation token. Stop invalidates earlier queued commands;
interrupted test threads cannot submit further motion. A timeout blocks motion
and requests cleanup, but cannot interrupt a vendor call already in progress or
confirm a physical motor stop. Shutdown keeps thread owners alive until Qt reports
that their threads have finished. Test timers call their control code directly in
the test thread, so hardware waits do not block the GUI.

Its force getters read the acquisition cache directly. `ForceAcquisition` owns
a separate serial thread, two DSCUSB connections, recent samples, software zero
offsets, and per-axis monotonic timestamps. It never controls motors.

During load and displacement control and manual autoload, the worker checks sample freshness
every 20 ms and before issuing motion commands. `force_fault()` latches a fault,
attempts to stop both axes, and signals the session to cancel the test. Reconnect
hardware to reset the latch. This software watchdog is not a real-time guarantee:
motor communication, Qt scheduling, and deceleration affect physical stop time.

The sensor reports internally calibrated grams. Software subtracts its zero,
applies the configured mechanical transmission factor, and converts to Newtons
only when requested. Internal DSCUSB calibration is never written.

Configuration loading supports old flat keys. Saving uses nested sections.
The Qt forms and `resources/config.example.json` are bundled inside the package
and accessed through `importlib.resources`. Writable `config.local.json` settings
live in the platform's user configuration directory (`platformdirs`), or at the
`BIAXIAL_CONFIG` override. Old checkout settings can be copied or selected through
that override; installation does not overwrite them.

The legacy `biaxial_testing_machine.motordaqinterface` import and constructor `ip_daq` argument
remain accepted for compatibility. Gantner acquisition and calibration are gone.

`MachineConfig.validate()` checks the selected test's numeric parameters before
session changes. Displacement targets also respect the existing 0-83 mm axis
travel, including the current zero position. No new vendor speed/load ratings
are assumed. Both test types report unexpected startup and timer errors through
`MechanicalTest._fail()`, latch a hardware fault, attempt both motor stops, and
finish without reporting a successful run. Disconnect attempts every resource
cleanup even when an earlier action fails.

Camera acquisition uses the existing `VideoThread` with Vimba X / VmbPy loaded
inside `run()`. Camera contexts, frame acquisition, and feature writes belong to
that worker. Frames are copied before their SDK context closes. Configuration
stores an optional `runtime.camera_id`; singleton discovery selects and retains
one identity, and ambiguous discovery reports the available IDs.

Tracking runs on native-resolution grayscale images. The view scales images with
letterboxing and maps clicks back to their native coordinates. ROI is configured
programmatically through `VideoThread.update_roi()` using native pixels; the camera
window has no drag-to-select ROI control. Camera reconnection or a resolution
change clears tracking references. Camera faults cancel a test only
when it uses video strain; the existing test failure path requests motor stops.
Gain/exposure requests are read under a lock and clamped to feature limits and
increments. UI controls receive only metadata and copied image arrays.

Application diagnostics use module loggers. `biaxial_testing_machine.__main__` configures console logging
at INFO level; detailed position and motion messages are available at DEBUG level.
