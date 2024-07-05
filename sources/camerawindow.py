# -*- coding: utf-8 -*-
"""
Created on Sun Jan  7 09:03:14 2024

@author: V
"""

import sys
import cv2
import math
import numpy as np
import time
import csv

from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QMessageBox
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap
from datetime import datetime

from vimba import *

from .markersdetection import markersDetection
     

class VideoThread(QThread):
    
    '''
    Class that constantly captures video from web camera and emits the signal to VideoWindow 
    to update the image.
    
    Has to flag to record first marks and to constantly track them.
    '''
    
    signal_change_pixmap = pyqtSignal(np.ndarray)
    signal_marks_recorded = pyqtSignal(list)
    signal_marks_coordinates = pyqtSignal(list)

    res_x_full = 1024
    res_y_full = 768

    
    def __init__(self):
        super().__init__()
     
        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continiously track marks
        
        self._execute = True
            
        self._roi_x1 = 0
        self._roi_y1 = 0
        self._roi_x2 = VideoThread.res_x_full
        self._roi_y2 = VideoThread.res_y_full

        self.__initVariables()

    def __initVariables(self):
        """
        Initialize variables to store tracks of marks and groups.
        """

        #datastructures to store tracks of marks
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

    def stop(self):
        #self.timer.stop()
        #self.cam._close()
        self._execute = False
        self.quit()
        
    def update_roi(self, x1,y1, x2, y2):
        """
        Update the region of interest (ROI) with the given coordinates.
        
        Args:
            x1 (int): The x-coordinate of the top-left corner of the ROI.
            y1 (int): The y-coordinate of the top-left corner of the ROI.
            x2 (int): The x-coordinate of the bottom-right corner of the ROI.
            y2 (int): The y-coordinate of the bottom-right corner of the ROI.
        """

        #Adjust to difference between VideoThread and VideoWindow resolution
        x1 = int(x1 * VideoThread.res_x_full/VideoWindow.res_x)
        y1 = int(y1 * VideoThread.res_y_full/VideoWindow.res_y)
        x2 = int(x2 * VideoThread.res_x_full/VideoWindow.res_x)
        y2 = int(y2 * VideoThread.res_y_full/VideoWindow.res_y)
        
        #The roi should have the same orientation independent on how user indicated it on the screen
        if x2 < x1:
            x = x2
            x2 = x1
            x1 = x
            
        if y2 < y1:
            y = y2
            y2 = y1
            y1 = y
            
        
        self._roi_x1 = x1
        self._roi_y1 = y1
        
        self._roi_x2 = x2
        self._roi_y2 = y2
        
        self._init_marks = True
        self.__initVariables()
        
    def _if_within_roi (self, p):
        """
        Check if the point p is within the active region of interest (ROI).
        
        Parameters:
        - p: a tuple representing the point coordinates (x, y)
        
        Returns:
        - True if the point is within the ROI, False otherwise
        """
        
        #check in the point p is wihtin active roi
        
        if p[0] > self._roi_x1 and p[1] > self._roi_y1 and p[0] < self._roi_x2 and p[1] < self._roi_y2:
            return True
        else:
            return False
        

    """
    def startStopTracking(self, flag):
        
        This function starts or stops tracking based on the value of the flag parameter.
        It takes in the flag parameter, which is a boolean to indicate whether to start or stop tracking.
        There are no return types specified for this function.
        
        if flag:
            print("Start tracking")
            self._track_marks = True  # Change the flag
            self._start_time = time.perf_counter()
            self._current_time = 0
        
        else:
            self._execute = False
            self.stop()

    """
    
    
    def run(self):
        """
        The run function controls the execution of the main camera loop. 
        It initializes the camera and sets its parameters, then enters a loop where it 
        continuously captures frames from the camera. 
        It processes each frame to detect markers and record their positions. 
    
        Algorithm:

        *Constantly receive frames and track marks
        *if self._init_marks: Record first marks
        *after continiosly record marks and send them to MechanicalTest
        """
        #self.cap = cv2.VideoCapture(0)
        
        img_track = np.zeros((VideoThread.res_y_full, VideoThread.res_x_full, 1), dtype=np.uint8)
        img_track.fill(0)
        self._time = []
        
        with Vimba.get_instance () as vimba:
            cams = vimba.get_all_cameras ()
            with cams [0] as cam:
                
                cam.Gain.set(15)
                cam.ExposureTime.set(700)
        
                while self._execute:
                    
                    t = time.perf_counter()
                    time.sleep(0.05)
                    frame = cam.get_frame ()
                    
                    if frame:
                        
                        img_cv = frame.as_opencv_image()
                        
                        #print("Time 1 {}".format(time.perf_counter()-t))
                        
                        #detect markers
                        img, coord_temp = markersDetection().detectMarkers(img_cv) #detection of Markers

                        #Draw all the markers
                        img = cv2.addWeighted(img_track, 1, img, 1, 0)
                        #Decrease the resolution to decrease amount of data sent via signal-slot mechanism
                        img = cv2.resize(img, (VideoWindow.res_x, VideoWindow.res_y))
                        self.signal_change_pixmap.emit(img)
                           
                        # True only one time to record initial position of the marks
                        if self._init_marks:

                            self.__initVariables()
                            
                            print("Start recording")
                            print("Point 1 {},  {}".format(self._roi_x1, self._roi_y1))
                            print("Point 2 {},  {}".format(self._roi_x2, self._roi_y2))

                            filtered_coord = [coord for coord in coord_temp if self._if_within_roi(coord)]

                            #We need to ensure constant point order on the image
                            #sort by y coord to separate upper and lower marks
                            sorted_y = sorted(filtered_coord, key=lambda x: x[1])
                            #split into upper and lower
                            upper_coord = sorted_y[:2]
                            lower_coord = sorted_y[2:]
                            #sort by x coord
                            upper_sorted = sorted(upper_coord, key=lambda x: x[0], reverse=False)
                            lower_sorted = sorted(lower_coord, key=lambda x: x[0], reverse=True)

                            sorted_coord = lower_sorted + upper_sorted

                            print ("Sorted: ")
                            print(sorted_coord)
                            n_marks = len(filtered_coord)


                            if n_marks == 4:

                                #allocate (x,y) in first empty sub(list)
                                i = 0
                                for coord in sorted_coord:
                                    #print a number on the image
                                    # Font type
                                    if self._if_within_roi(coord)  and i < len(self._marks_groups):
                                        self._marks_groups[i].append(coord)
                                        i += 1

                                        font = cv2.FONT_HERSHEY_SIMPLEX
                                        p = (int(coord[0]+10),int(coord[1])+10)
                                        cv2.putText(img, str(i), p, font, 1, 155, 2)


                                print("Marks count: {}".format(i))

                                self.signal_marks_recorded.emit(self._marks_groups)
                                #print("Recorded marks:")
                                #print(self._marks_groups)
                                self.stop()

                            else:
                                print("Wrong number of marks. Expected 4. Detected {}".format(n_marks))
                                      
                            #Switch to continous monitoring mode
                            self._init_marks = False
                            self._track_marks = True
                            self.signal_marks_coordinates([self._point1[-1],self._point2[-1],self._point3[-1],self._point4[-1]])

                        elif self._track_marks:
                            
                            #distribute marks in the groups
                            #go by all points and find a closest marker to it
                            for group in self._marks_groups:
                                if len(group) > 0:
                                    min_dist = 2000
                                    el = None
                                    #element - pair of (x,y) coordinates of a mark
                                    for element in coord_temp: 
                                        #last = gr[len(gr) - 1]
                                        dist = math.dist(element, group[-1])
                                        #print(dist)
                                        #print("dist {} and {} = {}".format(element, last, dist))
                                        if dist < min_dist and dist < 70:
                                            el = element
                                            min_dist = dist
                                    if el is not None:
                                        group.append(el)
                                        #print(gr)

                            self.signal_marks_coordinates([self._point1[-1],self._point2[-1],self._point3[-1],self._point4[-1]])

                          
                        
                        
                        
                        #draw track of the markers
                        for group in self._marks_groups:
                            lg = len(group)
                            if lg > 1:
                                cv2.line(img_track, group[lg-2], group[lg-1], 150, 1)
                   
                    
                        
                    
                        
                        print("One frame {}".format(time.perf_counter()-t))
                        
                        
                
                
                #Record to the file
                #self.writeDataToFile()
                self.quit()
            
                    

class VideoWindow(QWidget):
    signal_update_roi = pyqtSignal(int, int, int, int)
    #start_tracking_signal = pyqtSignal(bool)  # Signal to start tracking marks
    
    res_x = int(1024/2)
    res_y = int(768/2)
    

    def __init__(self, thread):
        

        """
        Initializes the video stream window and sets up the interface for displaying the video feed 
        and controlling the webcam. 
        """
        super().__init__()
        self.setWindowTitle("Video Stream")
        self._image_label = QLabel(self)
        self._image_label.resize(VideoWindow.res_x, VideoWindow.res_y)
        self._layout = QVBoxLayout()
        self._layout.addWidget(self._image_label)
        self.setLayout(self._layout)

        self._button_stop = QPushButton('Stop Webcam', self)
        self._button_stop.clicked.connect(self.stop_webcam)
        self._layout.addWidget(self._button_stop)
        
        '''
        self.button_tr_marks = QPushButton('Start tacking', self)
        self.button_tr_marks.clicked.connect(self._track_marks_clicked)
        self._layout.addWidget(self.button_tr_marks)
        '''
        self._start_point = self._end_point = None
        
        self._track_marks = False
         
        
        self.thread = thread
        #self.thread.change_pixmap_signal.connect(self.update_image)
        self.signal_update_roi.connect(self.thread.update_roi)
        
            
    def mousePressEvent(self, event):
        """
        Handle the mouse press event and update the start point, end point, and trigger an update.
        """
        self._start_point = event.pos()
        self._end_point = None
        self.update()

    def mouseMoveEvent(self, event):
        self._end_point = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        self._end_point = event.pos()
        
        print("start point {}".format(self._start_point))
        print("end point {}".format(self._end_point))
        self.signal_update_roi.emit(self._start_point.x(), self._start_point.y(), self._end_point.x(), self._end_point.y())

    def startStopTracking(self, flag):
        self.thread.startStopTracking(flag)
        
        self._track_marks = flag
        

    def stop_webcam(self):
        self.thread.stop()
        self.close()
        

    def update_image(self, cv_img):
        """
        Update the image displayed in the image label with the given cv_img.
        
        Args:
            cv_img: The OpenCV image to update the display with.
        """
        
        if self._start_point and self._end_point:
            start_p = (self._start_point.x(), self._start_point.y())
            end_p = (self._end_point.x(), self._end_point.y())
            if not self._track_marks:
                cv2.rectangle(cv_img, start_p, end_p, 170, 3)
        
        ch = 1
        
        if (2 == len(cv_img.shape)):
            h, w = cv_img.shape
        else:
            h, w, _ = cv_img.shape
        
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8)
        p = convert_to_Qt_format.scaled(VideoWindow.res_x, VideoWindow.res_y, Qt.AspectRatioMode.KeepAspectRatio)
        
 
        
        '''
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        '''
        
        qt_img = QPixmap.fromImage(p)
        self._image_label.setPixmap(qt_img)
        

    def stop_webcam(self):
        self.thread.stop()
        self.close()
        
    def closeEvent(self, event):
        self.stop_webcam()



if __name__ == '__main__':
    print("Started")
    app = QApplication(sys.argv)
    _video_thread = VideoThread()
    _video_window = VideoWindow(_video_thread)
    _video_thread.start()
    _video_window.show()
    sys.exit(app.exec())
