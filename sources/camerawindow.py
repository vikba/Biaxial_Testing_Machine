# -*- coding: utf-8 -*-
"""
Created on Sun Jan  7 09:03:14 2024

Updated on Oct 23, 2024

@description:
A PyQt6 application that captures video from a web camera using Vimba and OpenCV.
It allows users to define a region of interest (ROI), track markers within the ROI,
and adjust camera settings such as gain and exposure via sliders. The interface also
displays coordinate axes to indicate the origin of the coordinate plane.

@author: V
"""

import sys
import cv2
import math
import numpy as np
import time

from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
    QHBoxLayout, QSlider, QGroupBox, QRadioButton, QCheckBox, QButtonGroup
)
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtGui import QImage, QPixmap

from vimba import *


class VideoThread(QThread):
    """
    A QThread subclass that continuously captures video frames from a web camera
    and emits signals to update the image in the VideoWindow. It can initialize
    and track markers in the video stream.
    """

    signal_change_pixmap = pyqtSignal(np.ndarray)
    signal_markers_recorded = pyqtSignal(list)
    signal_markers_coordinates = pyqtSignal(list)

    RES_X_FULL = 1024
    RES_Y_FULL = 768

    def __init__(self):
        super().__init__()

        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continuously track marks

        self._init_points = None

        self._execute = True

        self._roi_x1 = 0
        self._roi_y1 = 0
        self._roi_x2 = VideoThread.RES_X_FULL
        self._roi_y2 = VideoThread.RES_Y_FULL

        self._initVariables()

        # Initialize gain and exposure values
        self._gain_value = 15
        self._exposure_value = 2000
        self._threshold_value = 70  # Default threshold

        self._apply_bg_correction = True
        self._use_simple_thresh = True
        self._center_method = 'bounding_box'


        self._cam = None  # Camera object

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
        Stops the video thread and releases resources.
        """
        self._execute = False
        self.quit()
        self.wait()
        self._cam = None

    @pyqtSlot()
    def reset_flags(self):
        """
        Resets flags for initializing and tracking marks.
        """
        self._execute = True
        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continuously track marks

    @pyqtSlot(list)
    def set_init_points(self, points):
        self._init_points = points
        self._init_marks = True

    def reset_countours(self):
        """
        Resets the contour image used for tracking markers.
        """
        self._img_track = np.zeros(
            (VideoThread.RES_Y_FULL, VideoThread.RES_X_FULL, 1), dtype=np.uint8
        )
        self._img_track.fill(0)

    @pyqtSlot(int, int, int, int)
    def update_roi(self, x1, y1, x2, y2):
        """
        Update the region of interest (ROI) with the given coordinates.

        Args:
            x1 (int): The x-coordinate of the top-left corner of the ROI.
            y1 (int): The y-coordinate of the top-left corner of the ROI.
            x2 (int): The x-coordinate of the bottom-right corner of the ROI.
            y2 (int): The y-coordinate of the bottom-right corner of the ROI.
        """
        # Adjust to difference between VideoThread and VideoWindow resolution
        x1 = int(x1 * VideoThread.RES_X_FULL / VideoWindow.RES_X)
        y1 = int(y1 * VideoThread.RES_Y_FULL / VideoWindow.RES_Y)
        x2 = int(x2 * VideoThread.RES_X_FULL / VideoWindow.RES_X)
        y2 = int(y2 * VideoThread.RES_Y_FULL / VideoWindow.RES_Y)

        # The roi should have the same orientation independent of how user indicated it on the screen
        if x2 < x1:
            x1, x2 = x2, x1

        if y2 < y1:
            y1, y2 = y2, y1

        self._roi_x1 = x1
        self._roi_y1 = y1

        self._roi_x2 = x2
        self._roi_y2 = y2

        self._init_marks = True
        self._initVariables()

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
        if flag:
            print("Start tracking")
            self._track_marks = True  # Change the flag
            self._start_time = time.perf_counter()
            self._current_time = 0
        else:
            self._execute = False
            self._track_marks = False
            self._init_marks = False
            self.stop()

    @pyqtSlot(str)
    def save_image(self, address):
        """
        Saves the current image to the specified address.

        Args:
            address (str): The file path to save the image.
        """
        print(f"Saving image with an address {address}")
        cv2.imwrite(address, self._img)

    @pyqtSlot(int)
    def set_gain(self, value):
        """
        Sets the camera gain to the specified value.

        Args:
            value (int): The gain value to set.
        """
        self._gain_value = value
        if self._cam is not None:
            self._cam.Gain.set(value)

    @pyqtSlot(int)
    def set_exposure(self, value):
        """
        Sets the camera exposure time to the specified value.

        Args:
            value (int): The exposure time value to set.
        """
        self._exposure_value = value
        if self._cam is not None:
            self._cam.ExposureTime.set(value)

    @pyqtSlot(bool)
    def set_apply_bg_correction(self, value):
        self._apply_bg_correction = value

    @pyqtSlot(bool)
    def set_use_simple_thresh(self, value):
        self._use_simple_thresh = value

    @pyqtSlot(str)
    def set_center_method(self, method):
        self._center_method = method

    def run(self):
        """
        The run function controls the execution of the main camera loop.
        It initializes the camera and sets its parameters, then enters a loop where it
        continuously captures frames from the camera.
        It processes each frame to detect markers and record their positions.
        """
        self._img_track = np.zeros(
            (VideoThread.RES_Y_FULL, VideoThread.RES_X_FULL, 1), dtype=np.uint8
        )
        self._img_track.fill(0)
        self._time = []

        with Vimba.get_instance() as vimba:
            cams = vimba.get_all_cameras()
            if len(cams) > 0:
                self._cam = cams[0]
                with self._cam:
                    self._cam.Gain.set(self._gain_value)
                    self._cam.ExposureTime.set(self._exposure_value)

                    while self._execute:
                        self._grab_frame()
                        QThread.msleep(100)
            else:
                print("No camera detected")
                self._execute = False

    def _grab_frame(self):
        if self._cam is not None and self._execute:
            frame = self._cam.get_frame()
            if frame:
                img_cv = frame.as_opencv_image()

                # Detect markers with current parameters
                processed_img, coord_temp = self._detectMarkers(
                    img_cv,
                    apply_bg_correction=self._apply_bg_correction,
                    use_simple_thresh=self._use_simple_thresh,
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
                #self._thresh_img = thresh_img

                # Emit both images for display
                img_resized = cv2.resize(combined_img, (VideoWindow.RES_X, VideoWindow.RES_Y))
                #thresh_resized = cv2.resize(thresh_img, (VideoWindow.RES_X, VideoWindow.RES_Y))
                self.signal_change_pixmap.emit(img_resized)

                # Handle marker initialization
                if self._init_marks and self._init_points:
                    sorted_coord = []
                    for target_point in self._init_points:
                        
                        # Find closest detected marker
                        min_dist = float('inf')
                        closest = None
                        for detected in coord_temp:
                            dist = math.dist(target_point, detected)
                            
                            if dist < min_dist and dist < 40:  # 100px max distance
                                min_dist = dist
                                closest = detected
                        if closest:
                            sorted_coord.append(closest)

                    print(self._init_points)
                    print(sorted_coord)

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
                        print("Could not find all the markers")

                    self._init_marks = False
                    self._init_points = None
                    

                elif self._track_marks:
                    # Distribute marks in the groups
                    # Go by all points and find the closest marker to it
                    for group in self._marks_groups:
                        if len(group) > 0:
                            min_dist = 2000
                            el = None
                            # element - pair of (x,y) coordinates of a mark
                            for element in coord_temp:
                                # last = gr[len(gr) - 1]
                                dist = math.dist(element, group[-1])
                                if dist < min_dist and dist < 10:
                                    el = element
                                    min_dist = dist
                            if el is not None:
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
        origin_y = self.RES_Y_FULL - 50  # Y position of origin (from bottom)
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

    def calculate_time(func):
        """
        Decorator to calculate and print the execution time of a function.
        """
        def inner1(*args, **kwargs):
            begin = time.time()
            func(*args, **kwargs)
            end = time.time()
            print(f"Total time taken in {func.__name__}: {end - begin:.4f} seconds")
        return inner1

    @calculate_time
    def example_decorated_function():
        pass  # Example usage

    @pyqtSlot(int)
    def set_threshold(self, value):
        """
        Sets the threshold value for marker detection.

        Args:
            value (int): The new threshold value.
        """
        self._threshold_value = value


    def _detectMarkers(
        self, 
        image, 
        apply_bg_correction=True, 
        use_simple_thresh=True, 
        thresh_val=100, 
        center_method='bounding_box'
    ):
        """
        Detects markers on a gray/white background.
        
        Args:
            image (np.ndarray): Input grayscale image.
            apply_bg_correction (bool): Whether to perform background correction.
            use_simple_thresh (bool): If True, use fixed thresholding; otherwise, use DoG approach.
            thresh_val (int): Fixed threshold value used if use_simple_thresh is True.
            center_method (str): Method to compute marker center. Options: 'moments' (contour moments) 
                or 'bounding_box' (bounding box center). Defaults to 'moments'.
        
        Returns:
            tuple:
                - res_img (np.ndarray): Image with markers drawn.
                - marks_groups (list): List of (x, y) coordinates for detected markers.
        """
        # Optional background correction
        if apply_bg_correction:
            bg = cv2.GaussianBlur(image, (0, 0), sigmaX=50, sigmaY=50)
            corrected = cv2.bitwise_not(cv2.subtract(bg, image))
        else:
            corrected = image.copy()
        
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
            if area < 150 or area > 10000:
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
        
        return res_img, marks_groups





class VideoWindow(QWidget):
    """
    The main window class that displays the video stream and controls the webcam.
    """

    signal_update_roi = pyqtSignal(int, int, int, int)
    signal_gain_changed = pyqtSignal(int)
    signal_exposure_changed = pyqtSignal(int)
    signal_threshold_changed = pyqtSignal(int)
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
        self._image_label.resize(VideoWindow.RES_X, VideoWindow.RES_Y)
        self._layout = QVBoxLayout()
        self._layout.addWidget(self._image_label)
        self.setLayout(self._layout)

        '''# Stop Webcam Button
        self._button_stop = QPushButton('Stop Webcam', self)
        self._button_stop.clicked.connect(self.stop_webcam)
        self._layout.addWidget(self._button_stop)'''

        self.init_button = QPushButton("Initialize Markers")
        self.init_button.clicked.connect(self.start_point_collection)
        self._layout.addWidget(self.init_button)

        self._start_point = self._end_point = None
        self._draw_rectangle = False

        # Initialize sliders and their labels
        self._init_sliders()

        # Connect signals
        self.thread = thread
        self.thread.signal_change_pixmap.connect(self.update_image)
        self.signal_update_roi.connect(self.thread.update_roi)
        self.signal_gain_changed.connect(self.thread.set_gain)
        self.signal_exposure_changed.connect(self.thread.set_exposure)
        self.signal_threshold_changed.connect(self.thread.set_threshold)
        self.signal_init_points.connect(self.thread.set_init_points)

        
        
        # Add point collection variables
        self.point_collection_mode = False
        self.clicked_points = []
        

    def _init_sliders(self):
        """
        Initializes the gain and exposure sliders along with their value labels.
        """
        # Gain Controls
        self._gain_label = QLabel('Gain:', self)
        self._gain_slider = QSlider(Qt.Orientation.Horizontal, self)
        self._gain_slider.setMinimum(0)
        self._gain_slider.setMaximum(24)
        self._gain_slider.setValue(15)
        self._gain_slider.valueChanged.connect(self.change_gain)

        self._gain_value_label = QLabel(str(self._gain_slider.value()), self)

        # Exposure Controls
        self._exposure_label = QLabel('Exposure:', self)
        self._exposure_slider = QSlider(Qt.Orientation.Horizontal, self)
        self._exposure_slider.setMinimum(100)
        self._exposure_slider.setMaximum(20000)
        self._exposure_slider.setValue(700)
        self._exposure_slider.valueChanged.connect(self.change_exposure)

        self._exposure_value_label = QLabel(str(self._exposure_slider.value()), self)

        # Layout for Gain Slider
        gain_layout = QHBoxLayout()
        gain_layout.addWidget(self._gain_label)
        gain_layout.addWidget(self._gain_slider)
        gain_layout.addWidget(self._gain_value_label)

        # Layout for Exposure Slider
        exposure_layout = QHBoxLayout()
        exposure_layout.addWidget(self._exposure_label)
        exposure_layout.addWidget(self._exposure_slider)
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
        self._threshold_slider.setValue(30)  # Default value
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
        self.center_method_group = QButtonGroup()
        self.bbox_radio = QRadioButton("Bounding Box Center")
        self.moments_radio = QRadioButton("Contour Moments")

        # Set defaults
        self.bg_correction_check.setChecked(True)
        self.simple_thresh_check.setChecked(True)
        self.bbox_radio.setChecked(True)

        # Layout
        params_layout = QVBoxLayout()
        params_layout.addWidget(self.bg_correction_check)
        params_layout.addWidget(self.simple_thresh_check)
        params_layout.addWidget(QLabel("Center Calculation:"))
        params_layout.addWidget(self.bbox_radio)
        params_layout.addWidget(self.moments_radio)
        self.detect_params_group.setLayout(params_layout)
        self._layout.addWidget(self.detect_params_group)

        # Connect signals
        self.bg_correction_check.toggled.connect(lambda v: self.thread.set_apply_bg_correction(v))
        self.simple_thresh_check.toggled.connect(lambda v: self.thread.set_use_simple_thresh(v))
        self.bbox_radio.toggled.connect(lambda: self.thread.set_center_method('bounding_box'))
        self.moments_radio.toggled.connect(lambda: self.thread.set_center_method('moments'))


    def start_point_collection(self):
        self.point_collection_mode = True
        self.clicked_points = []
        self._image_label.setText("Click four points in any order")

    def mouseReleaseEvent(self, event):
        if self.point_collection_mode:
            x = event.pos().x() * (VideoThread.RES_X_FULL / VideoWindow.RES_X)
            y = event.pos().y() * (VideoThread.RES_Y_FULL / VideoWindow.RES_Y)
            self.clicked_points.append((x, y))
            
            if len(self.clicked_points) == 4:
                self.point_collection_mode = False
                sorted_points = self.sort_points(self.clicked_points)
                self.signal_init_points.emit(sorted_points)
                print("4 clicks made")

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

    '''def mousePressEvent(self, event):
        """
        Handle the mouse press event and update the start point.

        Args:
            event (QMouseEvent): The mouse event.
        """
        self._start_point = event.pos()
        self._end_point = None
        self.update()

    def mouseMoveEvent(self, event):
        """
        Handle the mouse move event and update the end point.

        Args:
            event (QMouseEvent): The mouse event.
        """
        self._end_point = event.pos()
        self._draw_rectangle = True
        self.update()

    def mouseReleaseEvent(self, event):
        """
        Handle the mouse release event, update the end point, and emit the ROI signal.

        Args:
            event (QMouseEvent): The mouse event.
        """
        self._end_point = event.pos()
        print("Start point: {}".format(self._start_point))
        print("End point: {}".format(self._end_point))
        self.signal_update_roi.emit(
            self._start_point.x(),
            self._start_point.y(),
            self._end_point.x(),
            self._end_point.y()
        )
        self._draw_rectangle = False'''

    @pyqtSlot(int)
    def change_gain(self, value):
        """
        Slot to handle gain slider value change.

        Args:
            value (int): The new gain value.
        """
        self._gain_value_label.setText(str(value))
        self.signal_gain_changed.emit(value)

    @pyqtSlot(int)
    def change_exposure(self, value):
        """
        Slot to handle exposure slider value change.

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

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """
        Update the image displayed in the image label with the given cv_img.

        Args:
            cv_img (np.ndarray): The OpenCV image to update the display with.
        """
        if self._start_point and self._end_point and self._draw_rectangle:
            start_p = (self._start_point.x(), self._start_point.y())
            end_p = (self._end_point.x(), self._end_point.y())
            cv2.rectangle(cv_img, start_p, end_p, 170, 3)

        ch = 1

        if len(cv_img.shape) == 2:
            h, w = cv_img.shape
        else:
            h, w, _ = cv_img.shape

        bytes_per_line = ch * w
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
        self.thread.stop()
        self.close()

    def closeEvent(self, event):
        """
        Handles the window close event.

        Args:
            event (QCloseEvent): The close event.
        """
        self.stop_webcam()


if __name__ == '__main__':
    print("Started")
    app = QApplication(sys.argv)
    _video_thread = VideoThread()
    _video_window = VideoWindow(_video_thread)
    _video_thread.start()
    _video_window.show()
    sys.exit(app.exec())
