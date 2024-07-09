# Class Flags and Signals Information

## Class Name: `MechanicalTest`

### Internal Flags
- **self._execute**: Flag to continue executing loop of data aquisition
- **self._use_video**: Flag that shows that camera was initialized


### Signals Sent
- **signal_update_charts**: Sent to BiaxMainWindow with last values of read variables to update charts
- **signal_update_force_label**: Sent to BiaxMainWindow to update values of live force indicator
- **signal_start_stop_tracking**: Sent to VideoThread to start tracking the marks (they should be initialized before)
- **signal_make_photo**: Sent to VideoThread to make a photo of a sample before and after the test

### Signals Received
- **readForceLive**:
  - **Sender**: `BiaxMainWindow`
  - **Effect on Flags**: No
  - **Other Effects**: Controlled by QTimer. Emits signal to update force label in BiaxMainWindow.
- **init_markers**:
  - **Sender**: `VideoThread.signal_markers_recorded`
  - **Effect on Flags**: Sets `self._use_video` to `True` 
  - **Other Effects**: Initialized variables to store coordinates of the points
- **update_markers**:
  - **Sender**: `VideoThread.signal_markers_coordinates`
  - **Effect on Flags**: No
  - **Other Effects**: Writes points coordinates in a temporaty buffer
