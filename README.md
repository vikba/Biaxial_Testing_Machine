# Biaxial Testing Machine Control Application

PyQt6 desktop application for operating a biaxial testing machine, running displacement- and load-controlled tests, viewing live camera data, and exporting measured results.

## Overview

This project provides the GUI and control logic for a biaxial mechanical testing setup. The application combines:

- machine configuration loading and saving
- motor and DAQ coordination
- displacement-control and load-control test workflows
- live charting with `pyqtgraph`
- camera acquisition through Allied Vision Vimba
- result export to Excel workbooks

The codebase is hardware-oriented and depends on vendor SDKs in addition to standard Python packages.

## Repository Layout

- `main.py` - application entry point
- `sources/` - GUI, hardware, session, and test-control modules
- `resources/` - Qt `.ui` files and default configuration data
- `debug/` and `misc/` - auxiliary project files

## Requirements

### Python packages

- `PyQt6`
- `pyqtgraph`
- `numpy`
- `scipy`
- `opencv-python`
- `openpyxl`

### Hardware and vendor dependencies

- Zaber Motion SDK: `zaber-motion`
- Gantner Instruments API: `ginsapy`
- Allied Vision Vimba SDK / Python bindings

These hardware dependencies must be installed and accessible on the target machine before the full application can run.

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
pip install PyQt6 pyqtgraph numpy scipy opencv-python openpyxl zaber-motion
```

Install the remaining vendor-specific packages and SDKs separately:

- `ginsapy`
- Allied Vision Vimba runtime and Python bindings

## Running the Application

Start the GUI with:

```bash
python main.py
```

The application loads its default configuration from `resources/config_test.json`.

## Notes

- This project is intended for a configured lab workstation, not a generic desktop environment.
- Camera and motion-control features will not work unless the corresponding hardware SDKs are installed and connected.
- Test results are exported through `openpyxl`.

## License

This project is licensed under the GNU General Public License v3.0. See `LICENSE` for the full license text.

Third-party SDKs and hardware libraries may have their own separate license terms. You are responsible for complying with those terms when deploying or distributing systems built on this project.
