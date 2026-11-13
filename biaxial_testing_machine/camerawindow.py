# -*- coding: utf-8 -*-
"""
Created on Sun Jan  7 09:03:14 2024

Updated on Oct 23, 2024

@description:
A PyQt6 application that captures video from a web camera using Vimba X and OpenCV.
It tracks markers within a programmatically configured region of interest (ROI),
and adjusts camera settings such as gain and exposure through numeric controls. The interface also
displays coordinate axes to indicate the origin of the coordinate plane.

@author: V
"""

import logging
import sys
import cv2
import math
import numpy as np
import time
from threading import Event, Lock
from queue import SimpleQueue
from concurrent.futures import ThreadPoolExecutor

from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
    QHBoxLayout, QSlider, QGroupBox, QRadioButton, QCheckBox, QButtonGroup,
    QSpinBox, QDoubleSpinBox
)
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtGui import QImage, QPixmap


logger = logging.getLogger(__name__)


class VideoThread(QThread):
    """
    A QThread subclass that continuously captures video frames from a web camera
    and emits signals to update the image in the VideoWindow. It can initialize
    and track markers in the video stream.
    """

    signal_change_pixmap = pyqtSignal(np.ndarray)
    signal_markers_recorded = pyqtSignal(list)
    signal_markers_coordinates = pyqtSignal(list)
    signal_error = pyqtSignal(str)
    signal_camera_settings = pyqtSignal(dict)
    signal_image_saved = pyqtSignal(str)

    FRAME_TIMEOUT_MS = 500
    FRAME_GRAB_DELAY_MS = 100
    CAMERA_RECONNECT_DELAY_MS = 500
    MAX_CONSECUTIVE_TIMEOUTS = 5
    DISPLAY_MODE_OVERLAY = "overlay"
    DISPLAY_MODE_BINARIZED = "binarized"

    def __init__(self, camera_id=""):
        super().__init__()
        self._camera_id = camera_id.strip()
        self._stop = Event()
        self._settings_lock = Lock()
        self._commands = SimpleQueue()
        self._pending_settings = {}
        self._writer = None
        self._frame_shape = None
        self._camera_settings = {}
        self._applied_settings = {}
        self._pending_roi = None
        self._img = None

        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continuously track marks

        self._init_points = None

        self._execute = True

        self._roi_x1 = 0
        self._roi_y1 = 0
        self._roi_x2 = self._roi_y2 = 0

        self._initVariables()

        # Initialize gain and exposure values
        self._gain_value = 15
        self._exposure_value = 5000
        self._threshold_value = 70  # Default threshold

        self._apply_bg_correction = True
        self._use_simple_thresh = True
        self._invert_signal = False
        self._display_mode = VideoThread.DISPLAY_MODE_OVERLAY
        self._min_marker_area = 150
        self._max_marker_area = 10000
        self._center_method = 'bounding_box'


        self._cam = None  # Camera object
        self._consecutive_timeouts = 0

    def set_camera_id(self, camera_id):
        """Change selection only after acquisition has fully stopped."""
        if self.isRunning():
            raise RuntimeError("Stop camera acquisition before changing camera selection.")
        self._camera_id = camera_id.strip()

    def _initVariables(self):
        """
        Initialize variables to store tracks of marks and groups.
        """
        # Data structures to store tracks of marks
        self._marks_groups = []
        self._point1 = []
        self._point2 = []
        self._point3 = []
        self._point4 = []

        self._marks_groups.append(self._point1)
        self._marks_groups.append(self._point2)
        self._marks_groups.append(self._point3)
        self._marks_groups.append(self._point4)

        self._E11 = []
        self._E22 = []

        self._temp_p1 = None
        self._temp_p2 = None
        self._temp_p3 = None
        self._temp_p4 = None

    @pyqtSlot()
    def stop(self):
        """
        Stops the video thread and releases rebiaxial_testing_machine.
        """
        with self._settings_lock:
            self._execute = False
            self._stop.set()

    @pyqtSlot()
    def reset_flags(self):
        """
        Resets flags for initializing and tracking marks.
        """
        if self.isRunning():
            return
        self._commands = SimpleQueue()
        self._stop.clear()
        self._frame_shape = None
        self._img = None
        self._init_points = self._pending_roi = None
        self._initVariables()
        self._execute = True
        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continuously track marks

    @pyqtSlot(list)
    def set_init_points(self, points):
        self._commands.put(("initialize", tuple(map(tuple, points))))

    def reset_countours(self):
        """
        Resets the contour image used for tracking markers.
        """
        if self._frame_shape is not None:
            self._img_track = np.zeros(self._frame_shape, dtype=np.uint8)

    @pyqtSlot(int, int, int, int)
    def update_roi(self, x1, y1, x2, y2):
        """Queue a native-pixel ROI; all tracking resets happen in the camera thread."""
        self._commands.put(("roi", (x1, y1, x2, y2)))

    def _if_within_roi(self, p):
        """
        Check if the point p is within the active region of interest (ROI).

        Parameters:
            p (tuple): A tuple representing the point coordinates (x, y).

        Returns:
            bool: True if the point is within the ROI, False otherwise.
        """
        return self._roi_x1 < p[0] < self._roi_x2 and self._roi_y1 < p[1] < self._roi_y2

    @pyqtSlot(bool)
    def start_stop_tracking(self, flag):
        """
        Starts or stops tracking based on the value of the flag parameter.

        Args:
            flag (bool): Indicates whether to start or stop tracking.
        """
        self._commands.put(("tracking", flag))

    @pyqtSlot(str)
    def save_image(self, address):
        with self._settings_lock:
            accepting = not self._stop.is_set() and self.isRunning()
            if accepting:
                self._commands.put(("save", address))
        if not accepting:
            self.signal_error.emit(f"Could not save camera image {address}: camera is stopped")

    def _write_image(self, address, image):
        try:
            if not cv2.imwrite(address, image):
                raise OSError("Image writer returned failure")
            self.signal_image_saved.emit(address)
        except Exception as exc:
            self.signal_error.emit(f"Could not save camera image {address}: {exc}")

    def _apply_commands(self):
        """Called only by acquisition: own all tracking and processing changes here."""
        with self._settings_lock:
            settings, self._pending_settings = self._pending_settings, {}
        for name, value in settings.items():
            setattr(self, name, value)
        self._min_marker_area = min(self._min_marker_area, self._max_marker_area)
        while not self._commands.empty():
            command, value = self._commands.get()
            if command == "save":
                if self._img is None or self._writer is None:
                    self.signal_error.emit(f"Could not save camera image {value}: no current image")
                else:
                    self._writer.submit(self._write_image, value, self._img.copy())
            elif command == "tracking":
                if value and not all(self._marks_groups):
                    self.signal_error.emit("Initialize four camera markers before tracking.")
                else:
                    self._track_marks = value
                    self._start_time = time.perf_counter()
                    self._current_time = 0
            elif command == "initialize" and self._frame_shape is not None:
                if len(value) != 4 or len(set(value)) != 4:
                    self.signal_error.emit("Select four distinct marker positions.")
                elif self._track_marks:
                    self.signal_error.emit("Stop tracking before initializing markers again.")
                else:
                    self._init_points = value
            elif command == "roi" and self._frame_shape is not None:
                height, width = self._frame_shape
                x1, y1, x2, y2 = value
                x1, x2 = sorted((max(0, min(width, x1)), max(0, min(width, x2))))
                y1, y2 = sorted((max(0, min(height, y1)), max(0, min(height, y2))))
                if x1 < x2 and y1 < y2:
                    self._pending_roi = (x1, y1, x2, y2)

    @pyqtSlot(float)
    def set_gain(self, value):
        """
        Sets the camera gain to the specified value.

        Args:
            value (int): The gain value to set.
        """
        with self._settings_lock:
            self._gain_value = value

    @pyqtSlot(float)
    def set_exposure(self, value):
        """
        Sets the camera exposure time to the specified value.

        Args:
            value (int): The exposure time value to set.
        """
        with self._settings_lock:
            self._exposure_value = value

    @pyqtSlot(bool)
    def set_apply_bg_correction(self, value):
        with self._settings_lock:
            self._pending_settings["_apply_bg_correction"] = value

    @pyqtSlot(bool)
    def set_use_simple_thresh(self, value):
        with self._settings_lock:
            self._pending_settings["_use_simple_thresh"] = value

    @pyqtSlot(bool)
    def set_invert_signal(self, value):
        with self._settings_lock:
            self._pending_settings["_invert_signal"] = value

    @pyqtSlot(str)
    def set_display_mode(self, mode):
        with self._settings_lock:
            self._pending_settings["_display_mode"] = mode

    @pyqtSlot(int)
    def set_min_marker_area(self, value):
        with self._settings_lock:
            self._pending_settings["_min_marker_area"] = value

    @pyqtSlot(int)
    def set_max_marker_area(self, value):
        with self._settings_lock:
            self._pending_settings["_max_marker_area"] = value

    @pyqtSlot(str)
    def set_center_method(self, method):
        with self._settings_lock:
            self._pending_settings["_center_method"] = method

    def run(self):
        """Own the SDK, camera and feature access entirely in this thread."""
        self._writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera-image-writer")
        try:
            from vmbpy import VmbSystem, PixelFormat, VmbFeatureError
            with VmbSystem.get_instance() as vmb:
                last_error = None
                while not self._stop.is_set():
                    try:
                        self._cam = self._select_camera(vmb.get_all_cameras())
                        with self._cam:
                            logger.info("Camera connected: %s", self._cam.get_id())
                            for name, value in (("TriggerMode", "Off"), ("ExposureAuto", "Off"), ("GainAuto", "Off")):
                                try:
                                    feature = self._cam.get_feature_by_name(name)
                                except VmbFeatureError:
                                    continue
                                if feature.is_writeable():
                                    feature.set(value)
                            if PixelFormat.Mono8 in self._cam.get_pixel_formats():
                                self._cam.set_pixel_format(PixelFormat.Mono8)
                            self._applied_settings.clear()
                            self._camera_settings.clear()
                            self._consecutive_timeouts = 0
                            while not self._stop.is_set():
                                self._apply_camera_settings()
                                if self._grab_frame():
                                    last_error = None
                                self._stop.wait(self.FRAME_GRAB_DELAY_MS / 1000)
                    except Exception as exc:
                        if not self._stop.is_set() and str(exc) != last_error:
                            last_error = str(exc)
                            logger.warning("Camera acquisition failed: %s", exc)
                            self.signal_error.emit(last_error)
                    finally:
                        self._apply_commands()
                        self._cam = None
                        self._frame_shape = None
                        self._init_marks = self._track_marks = False
                        self._init_points = self._pending_roi = None
                        self._img = None
                        self._initVariables()
                        self.signal_camera_settings.emit({})
                    self._stop.wait(self.CAMERA_RECONNECT_DELAY_MS / 1000)
        except Exception as exc:
            self.signal_error.emit(f"Vimba X camera unavailable: {exc}. Install Vimba X and its matching VmbPy package.")
        finally:
            self.stop()
            self._apply_commands()
            self._writer.shutdown(wait=True)
            self._writer = None
            self._execute = False

    def _select_camera(self, cams):
        if self._camera_id:
            for camera in cams:
                if camera.get_id() == self._camera_id:
                    return camera
            raise RuntimeError(f"Camera {self._camera_id!r} was not discovered. Check Vimba X Viewer and network settings.")
        if len(cams) == 1:
            self._camera_id = cams[0].get_id()  # Keep this identity across reconnects.
            return cams[0]
        ids = ", ".join(camera.get_id() for camera in cams)
        raise RuntimeError(f"Configure runtime.camera_id; discovered cameras: {ids or 'none'}.")

    def _apply_camera_settings(self):
        from vmbpy import VmbFeatureError
        with self._settings_lock:
            requested = {"Gain": self._gain_value, "ExposureTime": self._exposure_value}
        changed = False
        for name, value in requested.items():
            if self._applied_settings.get(name) == value:
                continue
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")
            try:
                feature = self._cam.get_feature_by_name(name)
            except VmbFeatureError:
                self._applied_settings[name] = value
                continue
            if not feature.is_readable() or not feature.is_writeable():
                self._applied_settings[name] = value
                continue
            low, high = feature.get_range()
            if name == "ExposureTime":
                high = min(high, (self.FRAME_TIMEOUT_MS - 100) * 1000)
            if high < low:
                raise ValueError("Minimum camera exposure exceeds the acquisition timeout budget.")
            step = feature.get_increment()
            actual = min(high, max(low, value))
            if step:
                actual = low + math.floor((actual - low) / step) * step
            feature.set(actual)
            self._applied_settings[name] = value
            self._camera_settings[name] = (low, high, step or 1.0, feature.get())
            changed = True
        if changed:
            self.signal_camera_settings.emit(dict(self._camera_settings, ready=self._frame_shape is not None))

    def _grab_frame(self):
        self._apply_commands()
        from vmbpy import FrameStatus, PixelFormat, VmbTimeout
        if self._cam is None or self._stop.is_set():
            return False
        try:
            with self._cam.get_frame_with_context(timeout_ms=self.FRAME_TIMEOUT_MS) as frame:
                if frame.get_status() != FrameStatus.Complete:
                    raise RuntimeError("Incomplete camera frame. Check GigE packet size and bandwidth in Vimba X Viewer.")
                pixel_format = frame.get_pixel_format()
                if pixel_format == PixelFormat.Mono8:
                    img_cv = frame.as_opencv_image().copy()
                elif PixelFormat.Mono8 in pixel_format.get_convertible_formats():
                    img_cv = frame.convert_pixel_format(PixelFormat.Mono8).as_opencv_image().copy()
                elif pixel_format == PixelFormat.Bgr8 or PixelFormat.Bgr8 in pixel_format.get_convertible_formats():
                    converted = frame if pixel_format == PixelFormat.Bgr8 else frame.convert_pixel_format(PixelFormat.Bgr8)
                    img_cv = cv2.cvtColor(converted.as_opencv_image(), cv2.COLOR_BGR2GRAY)
                else:
                    raise ValueError(f"Unsupported camera pixel format: {pixel_format}.")
            img_cv = img_cv.reshape(img_cv.shape[:2])
            self._consecutive_timeouts = 0
        except VmbTimeout as exc:
            self._consecutive_timeouts += 1
            if self._track_marks or any(self._marks_groups) or self._consecutive_timeouts >= self.MAX_CONSECUTIVE_TIMEOUTS:
                raise RuntimeError("Camera acquisition timed out; reconnecting and clearing marker references.") from exc
            return False
        if self._stop.is_set():
            return False
        if img_cv.shape != self._frame_shape:
            tracking = self._track_marks or any(self._marks_groups)
            self._frame_shape = img_cv.shape
            self._roi_x1 = self._roi_y1 = 0
            self._roi_y2, self._roi_x2 = img_cv.shape
            self._init_marks = self._track_marks = False
            self._init_points = self._pending_roi = None
            self._initVariables()
            self.reset_countours()
            self.signal_camera_settings.emit(dict(self._camera_settings, ready=True))
            if tracking:
                raise RuntimeError("Camera resolution changed; initialize markers again.")
        with self._settings_lock:
            roi, self._pending_roi = self._pending_roi, None
            init_points, self._init_points = self._init_points, None
            self._init_marks = False
        if roi is not None:
            self._roi_x1, self._roi_y1, self._roi_x2, self._roi_y2 = roi
            self._init_marks = self._track_marks = False
            self._init_points = None
            self._initVariables()
            self.reset_countours()
            self.signal_error.emit("Camera ROI changed; initialize markers again.")
        # Detect markers with current parameters
        processed_img, binary_img, coord_temp = self._detectMarkers(
            img_cv,
            apply_bg_correction=self._apply_bg_correction,
            use_simple_thresh=self._use_simple_thresh,
            invert_signal=self._invert_signal,
            center_method=self._center_method,
            thresh_val=self._threshold_value
        )

        # Combine tracking visualization
        combined_img = cv2.addWeighted(self._img_track, 1, processed_img, 1, 0)

        # Draw marker numbers if available
        if len(self._point1) > 0:
            font = cv2.FONT_HERSHEY_SIMPLEX
            points = [
                self._point1[-1] if self._point1 else (0,0),
                self._point2[-1] if self._point2 else (0,0),
                self._point3[-1] if self._point3 else (0,0),
                self._point4[-1] if self._point4 else (0,0)
            ]
            for i, p in enumerate(points, 1):
                if p != (0,0):
                    cv2.putText(combined_img, str(i),
                            (int(p[0])+10, int(p[1])+10),
                            font, 1, 155, 2)

        # Draw coordinate system
        self._draw_coordinate_system(combined_img)

        # Store images for display
        self._img = combined_img
        self._binary_img = binary_img

        if self._display_mode == VideoThread.DISPLAY_MODE_BINARIZED:
            display_img = self._binary_img
        else:
            display_img = combined_img

        self.signal_change_pixmap.emit(display_img.copy())

        # Handle marker initialization
        if init_points and roi is None:
            self._initVariables()
            self.reset_countours()
            sorted_coord = []
            for target_point in init_points:

                # Find closest detected marker
                min_dist = float('inf')
                closest = None
                for detected in coord_temp:
                    dist = math.dist(target_point, detected)

                    if dist < min_dist and dist < 40:  # Maximum initialization distance in native pixels
                        min_dist = dist
                        closest = detected
                if closest and closest not in sorted_coord:
                    sorted_coord.append(closest)

            if (len(sorted_coord) == 4):
                # Assign to groups
                for i in range(4):
                    self._marks_groups[i].append(sorted_coord[i])

                # Emit initial positions
                self.signal_markers_recorded.emit([
                    self._point1[-1],
                    self._point2[-1],
                    self._point3[-1],
                    self._point4[-1]
                ])

            else:
                self.signal_error.emit("Could not find four distinct markers; initialize them again.")

        elif self._track_marks:
            if not all(self._marks_groups):
                raise RuntimeError("Camera markers were invalidated; initialize all four again.")
            # Distribute marks in the groups
            # Go by all points and find the closest marker to it
            for group in self._marks_groups:
                if len(group) > 0:
                    min_dist = 2000
                    el = None
                    # element - pair of (x,y) coordinates of a mark
                    for element in coord_temp:
                        dist = math.dist(element, group[-1])
                        if dist < min_dist and dist < 10:
                            el = element
                            min_dist = dist
                    if el is None:
                        raise RuntimeError("Camera marker lost; initialize all four again.")
                    group.append(el)

            self.signal_markers_coordinates.emit(
                [
                    self._point1[-1],
                    self._point2[-1],
                    self._point3[-1],
                    self._point4[-1]
                ]
            )

        # Draw track of the markers
        for group in self._marks_groups:
            lg = len(group)
            if lg > 1:
                cv2.line(
                    self._img_track,
                    tuple(int(x) for x in group[lg - 2]),
                    tuple(int(x) for x in group[lg - 1]),
                    150,
                    1
                )

        return True

    def _draw_coordinate_system(self, img):
        """
        Draws a coordinate system visualization in the bottom-left corner of the image.
        Includes X and Y axes with labels indicating the coordinate plane orientation.
        
        Args:
            img (np.ndarray): The image array to draw on (modified in-place)
        """
        # Coordinate system parameters
        arrow_length = 50  # Length of axis arrows in pixels
        origin_x = 50       # X position of origin (left padding)
        origin_y = img.shape[0] - 50  # Y position of origin (from bottom)
        color = 255         # White color (8-bit grayscale)
        thickness = 2       # Line thickness
        font_scale = 0.7    # Text size
        
        # Draw X-axis (horizontal arrow pointing right)
        cv2.arrowedLine(
            img, 
            (origin_x, origin_y), 
            (origin_x + arrow_length, origin_y), 
            color, 
            thickness, 
            tipLength=0.3
        )
        
        # Draw Y-axis (vertical arrow pointing up)
        cv2.arrowedLine(
            img, 
            (origin_x, origin_y), 
            (origin_x, origin_y - arrow_length), 
            color, 
            thickness, 
            tipLength=0.3
        )
        
        # Add axis labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # X-axis label ("Axis 1")
        cv2.putText(
            img, 
            'Axis 1', 
            (origin_x + arrow_length + 10, origin_y),  # Position right of X arrow
            font, 
            font_scale, 
            color, 
            thickness
        )
        
        # Y-axis label ("Axis 2")
        cv2.putText(
            img, 
            'Axis 2', 
            (origin_x - 30, origin_y - arrow_length - 10),  # Position above Y arrow
            font, 
            font_scale, 
            color, 
            thickness
        )

    @pyqtSlot(int)
    def set_threshold(self, value):
        """
        Sets the threshold value for marker detection.

        Args:
            value (int): The new threshold value.
        """
        with self._settings_lock:
            self._pending_settings["_threshold_value"] = value


    def _detectMarkers(
        self, 
        image, 
        apply_bg_correction=True, 
        use_simple_thresh=True, 
        invert_signal=False,
        thresh_val=100, 
        center_method='bounding_box'
    ):
        """
        Detects markers on a gray/white background.
        
        Args:
            image (np.ndarray): Input grayscale image.
            apply_bg_correction (bool): Whether to perform background correction.
            use_simple_thresh (bool): If True, use fixed thresholding; otherwise, use DoG approach.
            invert_signal (bool): Whether to invert the image before detection so bright
                markers on a dark background match the default detection polarity.
            thresh_val (int): Fixed threshold value used if use_simple_thresh is True.
            center_method (str): Method to compute marker center. Options: 'moments' (contour moments) 
                or 'bounding_box' (bounding box center). Defaults to 'moments'.
        
        Returns:
            tuple:
                - res_img (np.ndarray): Image with markers drawn.
                - cleaned (np.ndarray): Cleaned binary image used for contour detection.
                - marks_groups (list): List of (x, y) coordinates for detected markers.
        """
        detection_input = cv2.bitwise_not(image) if invert_signal else image

        # Optional background correction
        if apply_bg_correction:
            bg = cv2.GaussianBlur(detection_input, (0, 0), sigmaX=50, sigmaY=50)
            corrected = cv2.bitwise_not(cv2.subtract(bg, detection_input))
        else:
            corrected = detection_input.copy()
        
        # Choose between two thresholding approaches
        if use_simple_thresh:
            # Use a fixed threshold (without Otsu)
            _, thresh = cv2.threshold(corrected, thresh_val, 255, cv2.THRESH_BINARY_INV)
        else:
            # DoG approach
            blur_small = cv2.GaussianBlur(corrected, (7, 7), 0)
            blur_large = cv2.GaussianBlur(corrected, (21, 21), 0)
            dog = cv2.normalize(cv2.subtract(blur_large, blur_small), None, 0, 255, cv2.NORM_MINMAX)
            # Otsu thresholding
            _, thresh = cv2.threshold(dog, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            thresh = cv2.bitwise_not(thresh)
        
        # Clean up small artifacts.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours and process them
        cnts = cv2.findContours(cleaned, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        
        marks_groups = []
        res_img = image.copy()
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self._min_marker_area or area > self._max_marker_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if not self._if_within_roi((x + w / 2, y + h / 2)):
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * (area / (perimeter * perimeter))
            if circularity < 0.3:
                continue
            
            # Compute center coordinates based on selected method
            if center_method == 'bounding_box':
                x, y, w, h = cv2.boundingRect(c)
                cx = round(x + w / 2, 1)
                cy = round(y + h / 2, 1)

                cv2.rectangle(res_img, (x, y), (x + w, y + h), (255, 0, 0), 1)

            else:  # Default to 'moments'
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = round(M["m10"] / M["m00"], 1)
                    cy = round(M["m01"] / M["m00"], 1)
                else:
                    cx, cy = 0, 0

                # Draw contours and center (unchanged)
                cv2.drawContours(res_img, [c], -1, (0, 255, 0), 1)
            
            # Draw center
            cv2.circle(res_img, (int(cx), int(cy)), 3, (0, 0, 255), -1)
            marks_groups.append((cx, cy))
        
        return res_img, cleaned, marks_groups





class VideoWindow(QWidget):
    """
    The main window class that displays the video stream and controls the webcam.
    """

    signal_update_roi = pyqtSignal(int, int, int, int)
    signal_gain_changed = pyqtSignal(float)
    signal_exposure_changed = pyqtSignal(float)
    signal_threshold_changed = pyqtSignal(int)
    signal_display_mode_changed = pyqtSignal(str)
    signal_min_marker_area_changed = pyqtSignal(int)
    signal_max_marker_area_changed = pyqtSignal(int)
    signal_init_points = pyqtSignal(list)

    RES_X = 640  # Width of the display window
    RES_Y = 480  # Height of the display window

    def __init__(self, thread):
        """
        Initializes the video stream window and sets up the interface for displaying the video feed
        and controlling the webcam.

        Args:
            thread (VideoThread): The video thread that captures frames from the webcam.
        """
        super().__init__()
        self.setWindowTitle("Video Stream")
        self._image_label = QLabel(self)
        self._image_label.setFixedSize(VideoWindow.RES_X, VideoWindow.RES_Y)
        self._image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._frame_shape = None
        self._layout = QVBoxLayout()
        self._layout.addWidget(self._image_label)
        self.setLayout(self._layout)

        self.init_button = QPushButton("Initialize Markers")
        self.init_button.clicked.connect(self.start_point_collection)
        self.init_button.setEnabled(False)
        self._layout.addWidget(self.init_button)

        gain = 15
        exposure = 5000
        thread.set_gain(gain)
        thread.set_exposure(exposure)

        # Initialize sliders and their labels
        self._init_sliders(gain, exposure)

        # Connect signals
        self.thread = thread
        self.thread.signal_change_pixmap.connect(self.update_image)
        self.thread.signal_camera_settings.connect(self._update_camera_controls)
        self.thread.finished.connect(self.close)
        self.signal_update_roi.connect(self.thread.update_roi)
        self.signal_gain_changed.connect(self.thread.set_gain)
        self.signal_exposure_changed.connect(self.thread.set_exposure)
        self.signal_threshold_changed.connect(self.thread.set_threshold)
        self.signal_display_mode_changed.connect(self.thread.set_display_mode)
        self.signal_min_marker_area_changed.connect(self.thread.set_min_marker_area)
        self.signal_max_marker_area_changed.connect(self.thread.set_max_marker_area)
        self.signal_init_points.connect(self.thread.set_init_points)

        
        
        # Add point collection variables
        self.point_collection_mode = False
        self.clicked_points = []
        

    def _init_sliders(self, gain, exposure):
        """
        Initializes the gain and exposure sliders along with their value labels.
        """
        # Gain Controls
        self._gain_label = QLabel('Gain:', self)
        self._gain_control = QDoubleSpinBox(self)
        self._gain_control.setDecimals(3)
        self._gain_control.setEnabled(False)
        self._gain_control.setMinimum(0)
        self._gain_control.setMaximum(24)
        self._gain_control.setValue(gain)
        self._gain_control.valueChanged.connect(self.change_gain)

        self._gain_value_label = QLabel(str(self._gain_control.value()), self)

        # Exposure Controls
        self._exposure_label = QLabel('Exposure (us):', self)
        self._exposure_control = QDoubleSpinBox(self)
        self._exposure_control.setDecimals(3)
        self._exposure_control.setEnabled(False)
        self._exposure_control.setMinimum(100)
        self._exposure_control.setMaximum(20000)
        self._exposure_control.setValue(exposure)
        self._exposure_control.valueChanged.connect(self.change_exposure)

        self._exposure_value_label = QLabel(str(self._exposure_control.value()), self)

        # Layout for Gain Slider
        gain_layout = QHBoxLayout()
        gain_layout.addWidget(self._gain_label)
        gain_layout.addWidget(self._gain_control)
        gain_layout.addWidget(self._gain_value_label)

        # Layout for Exposure Slider
        exposure_layout = QHBoxLayout()
        exposure_layout.addWidget(self._exposure_label)
        exposure_layout.addWidget(self._exposure_control)
        exposure_layout.addWidget(self._exposure_value_label)

        # Combine both layouts into a vertical layout
        sliders_layout = QVBoxLayout()
        sliders_layout.addLayout(gain_layout)
        sliders_layout.addLayout(exposure_layout)

        self._layout.addLayout(sliders_layout)

        # Threshold Controls
        self._threshold_label = QLabel('Threshold:', self)
        self._threshold_slider = QSlider(Qt.Orientation.Horizontal, self)
        self._threshold_slider.setMinimum(0)
        self._threshold_slider.setMaximum(255)
        self._threshold_slider.setValue(70)  # Matches VideoThread default
        self._threshold_slider.valueChanged.connect(self.change_threshold)

        self._threshold_value_label = QLabel(str(self._threshold_slider.value()), self)

        # Layout for Threshold Slider
        threshold_layout = QHBoxLayout()
        threshold_layout.addWidget(self._threshold_label)
        threshold_layout.addWidget(self._threshold_slider)
        threshold_layout.addWidget(self._threshold_value_label)

        sliders_layout.addLayout(threshold_layout)  # Add to the main sliders layout

    
        # Detection Parameters Group
        self.detect_params_group = QGroupBox("Detection Parameters")
        self.bg_correction_check = QCheckBox("Apply BG Correction")
        self.simple_thresh_check = QCheckBox("Use Simple Threshold")
        self.invert_signal_check = QCheckBox("Invert Signal (White on Dark)")
        self.show_binarized_check = QCheckBox("Show Binarized Image")
        self.min_marker_area_spin = QSpinBox()
        self.max_marker_area_spin = QSpinBox()
        self.center_method_group = QButtonGroup()
        self.bbox_radio = QRadioButton("Bounding Box Center")
        self.moments_radio = QRadioButton("Contour Moments")

        self.center_method_group.addButton(self.bbox_radio)
        self.center_method_group.addButton(self.moments_radio)

        # Set defaults
        self.bg_correction_check.setChecked(True)
        self.simple_thresh_check.setChecked(True)
        self.bbox_radio.setChecked(True)
        self.min_marker_area_spin.setRange(1, 200000)
        self.max_marker_area_spin.setRange(1, 200000)
        self.min_marker_area_spin.setValue(150)
        self.max_marker_area_spin.setValue(10000)

        marker_area_layout = QHBoxLayout()
        marker_area_layout.addWidget(QLabel("Marker Area:"))
        marker_area_layout.addWidget(QLabel("Min"))
        marker_area_layout.addWidget(self.min_marker_area_spin)
        marker_area_layout.addWidget(QLabel("Max"))
        marker_area_layout.addWidget(self.max_marker_area_spin)

        # Layout
        params_layout = QVBoxLayout()
        params_layout.addWidget(self.bg_correction_check)
        params_layout.addWidget(self.simple_thresh_check)
        params_layout.addWidget(self.invert_signal_check)
        params_layout.addWidget(self.show_binarized_check)
        params_layout.addLayout(marker_area_layout)
        params_layout.addWidget(QLabel("Center Calculation:"))
        params_layout.addWidget(self.bbox_radio)
        params_layout.addWidget(self.moments_radio)
        self.detect_params_group.setLayout(params_layout)
        self._layout.addWidget(self.detect_params_group)

        # Connect signals
        self.bg_correction_check.toggled.connect(lambda v: self.thread.set_apply_bg_correction(v))
        self.simple_thresh_check.toggled.connect(lambda v: self.thread.set_use_simple_thresh(v))
        self.invert_signal_check.toggled.connect(lambda v: self.thread.set_invert_signal(v))
        self.show_binarized_check.toggled.connect(self._on_show_binarized_toggled)
        self.min_marker_area_spin.valueChanged.connect(self._on_min_marker_area_changed)
        self.max_marker_area_spin.valueChanged.connect(self._on_max_marker_area_changed)
        self.bbox_radio.toggled.connect(lambda: self.thread.set_center_method('bounding_box'))
        self.moments_radio.toggled.connect(lambda: self.thread.set_center_method('moments'))


    def start_point_collection(self):
        self.point_collection_mode = True
        self.clicked_points = []
        self.init_button.setText("Click four markers in the image")

    def mouseReleaseEvent(self, event):
        if not self.point_collection_mode or self._frame_shape is None:
            return
        pixmap = self._image_label.pixmap()
        if pixmap is None or pixmap.isNull():
            return
        point = self._image_label.mapFrom(self, event.position().toPoint())
        x = point.x() - (self._image_label.width() - pixmap.width()) / 2
        y = point.y() - (self._image_label.height() - pixmap.height()) / 2
        if not (0 <= x < pixmap.width() and 0 <= y < pixmap.height()):
            return  # Ignore clicks in the margins around a non-4:3 camera image.
        height, width = self._frame_shape
        self.clicked_points.append((x * width / pixmap.width(), y * height / pixmap.height()))
        if len(self.clicked_points) == 4:
            self.point_collection_mode = False
            self.init_button.setText("Initialize Markers")
            self.signal_init_points.emit(self.sort_points(self.clicked_points))

    def sort_points(self, points):
        # Sort by descending y (bottom points first)
        sorted_y = sorted(points, key=lambda p: p[1], reverse=True)
        bottom_points = sorted(sorted_y[:2], key=lambda p: p[0], reverse=True)  # Right first
        top_points = sorted(sorted_y[2:], key=lambda p: p[0])  # Left first
        return [bottom_points[0], bottom_points[1], top_points[0], top_points[1]]

    @pyqtSlot()
    def stop(self):
        """
        Closes the video window and stops the webcam.
        """
        self.close()

    @pyqtSlot(dict)
    def _update_camera_controls(self, settings):
        ready = settings.get("ready", False)
        self.init_button.setEnabled(ready)
        if not ready:
            self.init_button.setText("Initialize Markers")
            self.point_collection_mode = False
            self.clicked_points = []
            self._frame_shape = None
            self._image_label.clear()
        for name, control, label in (("Gain", self._gain_control, self._gain_value_label),
                                     ("ExposureTime", self._exposure_control, self._exposure_value_label)):
            control.setEnabled(name in settings)
            if name in settings:
                low, high, step, value = settings[name]
                control.blockSignals(True)
                control.setRange(low, high)
                control.setSingleStep(step)
                control.setValue(value)
                label.setText(f"{value:g}")
                control.blockSignals(False)

    @pyqtSlot(float)
    def change_gain(self, value):
        """
        Slot to handle gain control changes.

        Args:
            value (int): The new gain value.
        """
        self._gain_value_label.setText(str(value))
        self.signal_gain_changed.emit(value)

    @pyqtSlot(float)
    def change_exposure(self, value):
        """
        Slot to handle exposure control changes.

        Args:
            value (int): The new exposure value.
        """
        self._exposure_value_label.setText(str(value))
        self.signal_exposure_changed.emit(value)

    @pyqtSlot(int)
    def change_threshold(self, value):
        """
        Slot to handle threshold slider value change.

        Args:
            value (int): The new threshold value.
        """
        self._threshold_value_label.setText(str(value))
        self.signal_threshold_changed.emit(value)

    @pyqtSlot(bool)
    def _on_show_binarized_toggled(self, checked):
        mode = (
            VideoThread.DISPLAY_MODE_BINARIZED
            if checked else VideoThread.DISPLAY_MODE_OVERLAY
        )
        self.signal_display_mode_changed.emit(mode)

    @pyqtSlot(int)
    def _on_min_marker_area_changed(self, value):
        if value > self.max_marker_area_spin.value():
            self.max_marker_area_spin.setValue(value)
        self.signal_min_marker_area_changed.emit(value)

    @pyqtSlot(int)
    def _on_max_marker_area_changed(self, value):
        if value < self.min_marker_area_spin.value():
            self.min_marker_area_spin.setValue(value)
        self.signal_max_marker_area_changed.emit(value)

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """
        Update the image displayed in the image label with the given cv_img.

        Args:
            cv_img (np.ndarray): The OpenCV image to update the display with.
        """
        if self._frame_shape != cv_img.shape[:2]:
            self.point_collection_mode = False
            self.clicked_points = []
        self._frame_shape = cv_img.shape[:2]
        if len(cv_img.shape) == 2:
            h, w = cv_img.shape
        else:
            h, w, _ = cv_img.shape

        bytes_per_line = cv_img.strides[0]
        convert_to_Qt_format = QImage(
            cv_img.data,
            w,
            h,
            bytes_per_line,
            QImage.Format.Format_Grayscale8
        )
        p = convert_to_Qt_format.scaled(
            VideoWindow.RES_X,
            VideoWindow.RES_Y,
            Qt.AspectRatioMode.KeepAspectRatio
        )

        qt_img = QPixmap.fromImage(p)
        self._image_label.setPixmap(qt_img)

    def stop_webcam(self):
        """
        Stops the webcam and closes the window.
        """
        self.close()

    def closeEvent(self, event):
        """
        Handles the window close event.

        Args:
            event (QCloseEvent): The close event.
        """
        self.thread.stop()
        if self.thread.isRunning():
            event.ignore()
        else:
            event.accept()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    app = QApplication(sys.argv)
    _video_thread = VideoThread()
    _video_window = VideoWindow(_video_thread)
    _video_thread.start()
    _video_window.show()
    sys.exit(app.exec())
