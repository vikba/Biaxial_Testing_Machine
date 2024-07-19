# Class Flags and Signals Information

## Class Name: `MechanicalTest`

### Internal Flags
- **self._execute**: Flag to continue executing loop of data aquisition
- **self._use_video**: Flag that shows that camera was initialized (by `init_markers`)


### Signals Sent
- **signal_update_charts**: Sent to `BiaxMainWindow` with last values of read variables to update charts
- **signal_start_stop_tracking**: Sent to `VideoThread` to start tracking the marks (they should be initialized before)
- **signal_make_photo**: Sent to `VideoThread` to make a photo of a sample before and after the test

### Signals Slots
- **init_markers**:
  - **Sender**: `VideoThread.signal_markers_recorded`
  - **Effect on Flags**: Sets `self._use_video` to `True` 
  - **Other Effects**: Initialized variables to store coordinates of the points
- **update_markers**:
  - **Sender**: `VideoThread.signal_markers_coordinates`
  - **Effect on Flags**: No
  - **Other Effects**: Writes points coordinates in a temporaty buffer


## Class Name: `VideoThread`

### Internal Flags
- **self._init_marks**: Flag to record position of initial points
- **self._track_marks**: Flag to continiously track marks positions during the text


### Signals Sent
- **signal_change_pixmap**: Sent to `VideoWindow` to update the image 
- **signal_markers_recorded**: Sent to `MechanicalTest` to initialize first set of markers
- **signal_markers_coordinates**: Sent to `MechanicalTest` and recorded in temporaty variables that can be accessed when needed

### Signals Slots
- **update_roi**:
  - **Sender**: `VideoWindow.signal_update_roi`
  - **Effect on Flags**: Sets `self._init_marks` to `True`
  - **Other Effects**: 
- **start_stop_tracking**:
  - **Sender**: `MechanicalTest.signal_start_stop_tracking`
  - **Effect on Flags**: Sets `self._track_marks` to sent flag
  - **Other Effects**: 
- **save_image**:
  - **Sender**: `MechanicalTest.signal_make_photo`
  - **Effect on Flags**: 
  - **Other Effects**: Saves current pixelmap with sent address


  ## Class Name: `VideoWindow`

### Internal Flags
- **self._draw_rectangle**: Flag to draw rectangle only if left mouse button is pressed

### Signals Sent
- **signal_update_roi**: Sent to `VideoThread` to ROI of markers detections

### Signals Slots
- **update_image**:
  - **Sender**: `VideoThread.signal_change_pixmap`
  - **Effect on Flags**: 
  - **Other Effects**: Updates pixelmap for visualization


