# Application structure

`main.py` creates the Qt main window. `TestSessionController` owns the hardware
interface and active test; load and displacement tests share `MechanicalTest`
for recording, export, and stop behavior.

`MotorDAQInterface` dispatches motor commands to `HardwareWorker` in a Qt thread.
Its force getters read the acquisition cache directly. `ForceAcquisition` owns
a separate serial thread, two DSCUSB connections, recent samples, software zero
offsets, and per-axis monotonic timestamps. It never controls motors.

During load control and manual autoload, the worker checks sample freshness
every 20 ms and before issuing motion commands. `force_fault()` latches a fault,
attempts to stop both axes, and signals the session to cancel the test. Reconnect
hardware to reset the latch. This software watchdog is not a real-time guarantee:
motor communication, Qt scheduling, and deceleration affect physical stop time.

The sensor reports internally calibrated grams. Software subtracts its zero,
applies the configured mechanical transmission factor, and converts to Newtons
only when requested. Internal DSCUSB calibration is never written.

Configuration loading supports old flat keys. Saving uses nested sections.
`resources/config.example.json` is portable; `config.local.json` contains the
workstation's settings and is ignored by Git.

The legacy `sources.motordaqinterface` import and constructor `ip_daq` argument
remain accepted for compatibility. Gantner acquisition and calibration are gone.
