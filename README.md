# Biaxial Testing Machine Control Application

PyQt6 desktop application for operating a biaxial testing machine, running displacement- and load-controlled tests, viewing live camera data, and exporting measured results.

## Overview

This project provides the GUI and control logic for a biaxial mechanical testing setup. The application combines:

- machine configuration loading and saving
- motor and DAQ coordination
- displacement-control and load-control test workflows
- live charting with `pyqtgraph`
- camera acquisition through Allied Vision Vimba X / VmbPy
- result export to Excel workbooks

The codebase is hardware-oriented and depends on vendor SDKs in addition to standard Python packages.

## Repository Layout

- `main.py` - application entry point
- `sources/` - GUI, hardware, session, and test-control modules
- `resources/` - Qt `.ui` files and default configuration data
- `tests/` - hardware-free checks
- `docs/` - architecture notes

## Requirements

### Python packages

- `pyserial`
- `PyQt6`
- `pyqtgraph`
- `numpy`
- `opencv-python`
- `openpyxl`

### Hardware and vendor dependencies

- Zaber Motion SDK: `zaber-motion`
- DSCUSB USB/virtual COM drivers
- Allied Vision Vimba X SDK and its matching VmbPy Python package (optional unless using a camera)

Install the motor and force-sensor dependencies on the lab workstation. Camera support is optional: a missing Vimba X installation produces a camera-specific error when acquisition is requested.

## Installation

Clone the repository:

```bash
git clone https://github.com/vikba/Biaxial_Testing_Machine.git
cd Biaxial_Testing_Machine
```

Create and activate a virtual environment, then install the Python dependencies you use in your environment:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Install the remaining vendor-specific packages and SDKs separately:

- Allied Vision Vimba X runtime, GigE transport layer/driver, and matching VmbPy wheel

## Running the Application

Start the GUI with:

```bash
python main.py
```

Use Python 3.10 or newer. The application loads `resources/config.local.json` if
present, otherwise `resources/config.example.json`. Saving settings writes only
the ignored local file; sample exports still include their own configuration.
Copy the example to the local filename to configure a fresh checkout. Set the
motor IP, both sensor serials, and output folder for your workstation.
Older flat configuration files remain readable; new saves use structured sections.

## Vimba X camera

This application targets Vimba X (`vmbpy`); the old Vimba/FireWire Python API is
no longer used. Follow Allied Vision's [Python installation instructions](https://docs.alliedvision.com/vimba_x/pythonAPIManual.html)
to install the VmbPy wheel matching your SDK into the same virtual environment
as the application. Updating the SDK does not update that Python package automatically.

First confirm continuous acquisition in **Vimba X Viewer**. Configure the GigE
camera and its Ethernet adapter on compatible subnets, and check the GigE filter
driver and packet size in Viewer. The application does not modify network settings.
See the [Viewer guide](https://docs.alliedvision.com/vimba_x/viewerGuide.html).

Set `runtime.camera_id` in your existing `resources/config.local.json` to the ID
shown in Viewer, for example `"DEV_000A47000000"`. An empty ID automatically
selects the only discovered camera. With multiple cameras, acquisition reports
their IDs and requires explicit configuration. Reconnection retains the selected
identity and never silently switches to another camera.

Images are processed in native camera pixels and displayed with preserved aspect
ratio. Click **Initialize Markers**, then select four distinct markers in the image;
clicks in the surrounding margins are ignored. Initialize markers for each new
test. Pixel thresholds, marker area limits, and maximum tracking movement are
measured in native pixels and may need adjustment for a different resolution or lens.

The app prefers `Mono8`; other formats are converted to `Mono8`, or through `Bgr8`
to grayscale if supported by the SDK. Gain and exposure controls use camera-reported
ranges and increments, with SDK writes performed only in the acquisition thread.
Exposure is capped at 400,000 microseconds to leave margin within the 500 ms frame
acquisition timeout. Preview acquisition is limited to approximately 10 frames/s
before image-processing time; this is not synchronized to force acquisition.

Stopping returns immediately while the worker releases its camera context. The
window remains alive until that worker exits. A vendor operation that stalls
outside frame acquisition can still delay final shutdown. Camera errors invalidate
marker references; an active test using camera strain is failed through the
existing motor-stop path. Preview can reconnect, but markers must be initialized
again. Tests that do not use camera strain are not canceled by camera faults.

Hardware-free checks use simulated VmbPy objects with real Qt/OpenCV. No physical
camera, SDK runtime version, or camera model has been validated in this environment.
Before lab use, verify these on the target workstation:

1. Record camera model, firmware, Vimba X, VmbPy, and Python versions.
2. Check live preview, gain/exposure, image saving, ROI, and marker selection.
3. Verify a known displacement and compare image-based strain with expectations.
4. Repeat camera start/stop and disconnect/reconnect the Ethernet cable.
5. During a low-load camera-based test, remove the cable and confirm test failure
   and both motor-stop attempts; reinitialize markers after reconnection.

## DSCUSB force acquisition

Set hardware identity and acquisition settings in the local JSON `force` section:

- `axis1_serial`: `"0017835720"`
- `axis2_serial`: `"0017835697"` (change this to assign the second axis)
- `axis1_transmission_factor`, `axis2_transmission_factor`: default `0.5`, preserving
  the previous force `/2` correction. Confirm these against the pulley geometry.
- `timeout`: serial read/write timeout in seconds, default `0.075`.
- `max_data_age`: maximum valid sample age in seconds, default `0.25`.

Ports are discovered by querying device serials (`65536 * SERH + SERL`), never by
COM number. Close DSC Toolkit before connecting. Both devices use 115200 baud,
8N1 and `!001:SYS?\r`. SYS is already calibrated in grams; Zero only stores a
software offset. Reported grams are `(SYS - zero) * transmission_factor`;
Newtons are grams times `9.80665 / 1000`. No calibration commands are sent.
The obsolete `ip_daq` argument/key is accepted for compatibility and ignored.

Acquisition runs separately from motor commands. Force getters read cached
samples; unavailable/stale samples raise `ForceDataError`. During load or displacement control
or autoload a 20 ms watchdog stops both motors when either channel exceeds the
age limit. A force fault cancels the session and blocks further motion until
hardware is disconnected and reconnected. Actual stop latency also includes
motor communication and deceleration; verify it on the machine.

Smoke test (start with motors stationary and a low test load):

1. Install `pyserial` and DSCUSB drivers, connect both devices, and confirm live readings.
2. Apply a known weight to each axis separately. Verify identity, sign, and the
   transmission factors. Reconnect using different USB ports and repeat.
3. Press Zero, add/remove a known weight, and switch grams/Newtons. Confirm the
   expected factor and that DSC Toolkit still shows unchanged internal calibration.
4. Run a low-load test, then disconnect either DSCUSB. Confirm both motors stop,
   the test ends with an error, and motion stays blocked until reconnect.
   Repeat for the other sensor and manual autoload; measure the stop latency.
5. Stop/disconnect normally and confirm the ports can reopen without restarting.

Run simulated protocol/cache/stop checks with `python -m unittest discover -s tests`.
Protocol checks mock Qt and vendor dependencies. Lifecycle checks use real Qt
threads with simulated hardware to verify responsive commands, cancellation,
timeouts, and failure cleanup. Install the Python dependencies above to run those
checks; they are skipped if those dependencies are unavailable. These checks do
not replace hardware verification.
Protocol reference: [Mantracourt DSCUSB Advanced Manual](https://us.mantracourt.com/wp-content/uploads/2017/10/DSCUSB_Advanced_Manual.pdf).

## Notes

- This project is intended for a configured lab workstation, not a generic desktop environment.
- Camera and motion-control features will not work unless the corresponding hardware SDKs are installed and connected.
- Test results are exported through `openpyxl`.

## License

This project is licensed under the GNU General Public License v3.0. See `LICENSE` for the full license text.

Third-party SDKs and hardware libraries may have their own separate license terms. You are responsible for complying with those terms when deploying or distributing systems built on this project.
