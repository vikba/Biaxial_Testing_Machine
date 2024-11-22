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
    QHBoxLayout, QSlider
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

        self._execute = True

        self._roi_x1 = 0
        self._roi_y1 = 0
        self._roi_x2 = VideoThread.RES_X_FULL
        self._roi_y2 = VideoThread.RES_Y_FULL

        self._initVariables()

        # Initialize gain and exposure values
        self._gain_value = 15
        self._exposure_value = 700

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
        """
        Captures a frame from the camera, processes it, and emits the signal to update the image.
        """
        if self._cam is not None and self._execute:
            frame = self._cam.get_frame()
            if frame:
                img_cv = frame.as_opencv_image()

                # Detect markers
                img, coord_temp = self._detectMarkers(img_cv)  # Detection of Markers

                # Draw all the markers
                img = cv2.addWeighted(self._img_track, 1, img, 1, 0)

                # Draw point numbers on image
                if len(self._point1) > 0:
                    font = cv2.FONT_HERSHEY_SIMPLEX

                    p = (int(self._point1[-1][0] + 10), int(self._point1[-1][1] + 10))
                    cv2.putText(img, "1", p, font, 1, 155, 2)
                    p = (int(self._point2[-1][0] + 10), int(self._point2[-1][1] + 10))
                    cv2.putText(img, "2", p, font, 1, 155, 2)
                    p = (int(self._point3[-1][0] + 10), int(self._point3[-1][1] + 10))
                    cv2.putText(img, "3", p, font, 1, 155, 2)
                    p = (int(self._point4[-1][0] + 10), int(self._point4[-1][1] + 10))
                    cv2.putText(img, "4", p, font, 1, 155, 2)

                # Variable to be accessed in other functions
                self._img = img

                # Draw X and Y arrows in the left bottom corner
                arrow_length = 50  # Length of the arrows in pixels

                # Coordinates for the origin point (left bottom corner)
                origin_x = 50
                origin_y = VideoThread.RES_Y_FULL - 50

                # Draw X axis arrow
                cv2.arrowedLine(
                    img,
                    (origin_x, origin_y),
                    (origin_x + arrow_length, origin_y),
                    255,
                    2,
                    tipLength=0.3
                )

                # Draw Y axis arrow
                cv2.arrowedLine(
                    img,
                    (origin_x, origin_y),
                    (origin_x, origin_y - arrow_length),
                    255,
                    2,
                    tipLength=0.3
                )

                # Label the axes
                cv2.putText(
                    img,
                    'Axis 1',
                    (origin_x + arrow_length + 10, origin_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    255,
                    2
                )
                cv2.putText(
                    img,
                    'Axis 2',
                    (origin_x, origin_y - arrow_length - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    255,
                    2
                )

                # Decrease the resolution to decrease amount of data sent via signal-slot mechanism
                img_resized = cv2.resize(img, (VideoWindow.RES_X, VideoWindow.RES_Y))
                self.signal_change_pixmap.emit(img_resized)

                # True only one time to record initial position of the marks
                if self._init_marks:
                    self._initVariables()

                    print("Start recording")
                    print("Point 1 {},  {}".format(self._roi_x1, self._roi_y1))
                    print("Point 2 {},  {}".format(self._roi_x2, self._roi_y2))

                    filtered_coord = [coord for coord in coord_temp if self._if_within_roi(coord)]

                    # We need to ensure constant point order on the image
                    # Sort by y coord to separate upper and lower marks
                    sorted_y = sorted(filtered_coord, key=lambda x: x[1])
                    # Split into upper and lower
                    upper_coord = sorted_y[:2]
                    lower_coord = sorted_y[2:]
                    # Sort by x coord
                    upper_sorted = sorted(upper_coord, key=lambda x: x[0], reverse=False)
                    lower_sorted = sorted(lower_coord, key=lambda x: x[0], reverse=True)

                    sorted_coord = lower_sorted + upper_sorted

                    print("Sorted: ")
                    print(sorted_coord)
                    n_marks = len(filtered_coord)

                    if n_marks == 4:
                        # Allocate (x,y) in first empty sublist
                        i = 0
                        for coord in sorted_coord:
                            # Print a number on the image
                            # Font type
                            if self._if_within_roi(coord) and i < len(self._marks_groups):
                                self._marks_groups[i].append(coord)
                                i += 1

                        print("Marks count: {}".format(i))

                        self.signal_markers_recorded.emit(
                            [
                                self._point1[-1],
                                self._point2[-1],
                                self._point3[-1],
                                self._point4[-1]
                            ]
                        )
                        self._init_marks = False
                    else:
                        print("Wrong number of marks. Expected 4. Detected {}".format(n_marks))

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
                                if dist < min_dist and dist < 70:
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

    def _detectMarkers(self, image):
        """
        Detects markers in the given image and returns the resulting image with markers
        drawn and the coordinates of the detected markers.

        Args:
            image (np.ndarray): The input image.

        Returns:
            tuple:
                - res_img (np.ndarray): The image with detected markers drawn.
                - marks_groups (list): List of coordinates of detected markers.
        """
        # Apply median blur to reduce noise
        blur = cv2.medianBlur(image, 3)
        blur_strong = cv2.medianBlur(image, 71)

        # Invert the blurred images
        blur_inv = cv2.bitwise_not(blur)
        blur_strong_inv = cv2.bitwise_not(blur_strong)

        # Subtract the strong blur from the regular blur
        subtract_image = cv2.subtract(blur_inv, blur_strong_inv)

        # Apply thresholding to obtain binary image
        ret, thresh = cv2.threshold(
            subtract_image,
            30,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # Define morphological kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations=3)

        # Invert the image back
        opening = ~opening

        # Find contours
        cnts = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        marks_groups = []
        res_img = image.copy()

        for c in cnts:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * (area / (perimeter * perimeter))

            if 0.5 < circularity and 50 < area < 4000:
                # Find center with image moments
                M = cv2.moments(c)

                if M["m00"] != 0:
                    x = round(M["m10"] / M["m00"], 1)
                    y = round(M["m01"] / M["m00"], 1)
                else:
                    x, y = 0, 0  # Assign default value if m00 is zero

                # Draw contours and center point
                cv2.drawContours(res_img, [c], -1, (0, 255, 0), 1)
                cv2.circle(res_img, (int(x), int(y)), 3, (0, 0, 255), -1)

                marks_groups.append((x, y))

        return res_img, marks_groups


class VideoWindow(QWidget):
    """
    The main window class that displays the video stream and controls the webcam.
    """

    signal_update_roi = pyqtSignal(int, int, int, int)
    signal_gain_changed = pyqtSignal(int)
    signal_exposure_changed = pyqtSignal(int)

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

        # Stop Webcam Button
        self._button_stop = QPushButton('Stop Webcam', self)
        self._button_stop.clicked.connect(self.stop_webcam)
        self._layout.addWidget(self._button_stop)

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

    @pyqtSlot()
    def stop(self):
        """
        Closes the video window and stops the webcam.
        """
        self.close()

    def mousePressEvent(self, event):
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
        self._draw_rectangle = False

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
