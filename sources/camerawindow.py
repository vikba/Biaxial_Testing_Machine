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

from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt6.QtGui import QImage, QPixmap

from vimba import *
     

class VideoThread(QThread):
    
    '''
    Class that constantly captures video from web camera and emits the signal to VideoWindow 
    to update the image.
    
    Has to flag to record first marks and to constantly track them.
    '''
    
    signal_change_pixmap = pyqtSignal(np.ndarray)
    signal_markers_recorded = pyqtSignal(list)
    signal_markers_coordinates = pyqtSignal(list)

    RES_X_FULL = 1024
    RES_Y_FULL = 768

    
    def __init__(self):
        super().__init__()
     
        self._init_marks = False  # Initialize first set of marks
        self._track_marks = False  # Continiously track marks
        
        self._execute = True
            
        self._roi_x1 = 0
        self._roi_y1 = 0
        self._roi_x2 = VideoThread.RES_X_FULL
        self._roi_y2 = VideoThread.RES_Y_FULL

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

        self._temp_p1 = None
        self._temp_p2 = None
        self._temp_p3 = None
        self._temp_p4 = None

    def stop(self):
        #self.timer.stop()
        #self.cam._close()
        #self._timer_video.stop()
        self._execute = False
        self.quit()
        
    @pyqtSlot(int, int, int, int)
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
        x1 = int(x1 * VideoThread.RES_X_FULL/VideoWindow.RES_X)
        y1 = int(y1 * VideoThread.RES_Y_FULL/VideoWindow.RES_Y)
        x2 = int(x2 * VideoThread.RES_X_FULL/VideoWindow.RES_X)
        y2 = int(y2 * VideoThread.RES_Y_FULL/VideoWindow.RES_Y)
        
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
        
    @pyqtSlot(bool)
    def start_stop_tracking(self, flag):
        
        """This function starts or stops tracking based on the value of the flag parameter.
        It takes in the flag parameter, which is a boolean to indicate whether to start or stop tracking.
        There are no return types specified for this function."""
        
        if flag:
            print("Start tracking")
            self._track_marks = True  # Change the flag
            self._start_time = time.perf_counter()
            self._current_time = 0
        
        else:
            self._execute = False
            self.stop()

    
    @pyqtSlot(str)
    def save_image(self, address):
        cv2.imwrite(address, self._img)

    
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
        
        
        self.__img_track = np.zeros((VideoThread.RES_Y_FULL, VideoThread.RES_X_FULL, 1), dtype=np.uint8)
        self.__img_track.fill(0)
        self._time = []
        
        with Vimba.get_instance () as vimba:
            cams = vimba.get_all_cameras ()
            with cams [0] as cam:
                
                cam.Gain.set(15)
                cam.ExposureTime.set(700)

                while self._execute:
                    t = time.perf_counter()
                    self.__grab_frame(cam)
                    QThread.msleep(100)
                    #print("One frame {}".format(time.perf_counter()-t))

                '''self._timer_video = QTimer()
                self._timer_video.timeout.connect(lambda: self.__grab_frame(cam))
                self._timer_video.start(1)  # Grab a frame every 300ms (about 3.3fps)'''

                
                #Record to the file
                #self.writeDataToFile()
                self.quit()

    #Decorator to calculate execution time
    def calculate_time(func):
        # added arguments inside the inner1,
        # if function takes any arguments,
        # can be added like this.
        def inner1(*args, **kwargs):

            # storing time before function execution
            begin = time.time()
            
            func(*args, **kwargs)

            # storing time after function execution
            end = time.time()
            print("Total time taken in : ", func.__name__, end - begin)

        return inner1
    
    #@calculate_time
    def __grab_frame(self, cam):
        if cam is not None and self._execute:

            frame = cam.get_frame ()

            
            if frame:
                
                img_cv = frame.as_opencv_image()
                
                #print("Time 1 {}".format(time.perf_counter()-t))
                
                #detect markers
                img, coord_temp = self.__detectMarkers(img_cv) #detection of Markers

                #Draw all the markers
                img = cv2.addWeighted(self.__img_track, 1, img, 1, 0)

                #Draw point numbers on image
                if 0 < len(self._point1):
                    font = cv2.FONT_HERSHEY_SIMPLEX

                    p = (int(self._point1[-1][0]+10),int(self._point1[-1][1]+10))
                    cv2.putText(img, "1", p, font, 1, 155, 2)
                    p = (int(self._point2[-1][0]+10),int(self._point2[-1][1]+10))
                    cv2.putText(img, "2", p, font, 1, 155, 2)
                    p = (int(self._point3[-1][0]+10),int(self._point3[-1][1]+10))
                    cv2.putText(img, "3", p, font, 1, 155, 2)
                    p = (int(self._point4[-1][0]+10),int(self._point4[-1][1]+10))
                    cv2.putText(img, "4", p, font, 1, 155, 2)

                #Variable to be accessed in other functions
                self._img = img
                
                #Decrease the resolution to decrease amount of data sent via signal-slot mechanism
                img = cv2.resize(img, (VideoWindow.RES_X, VideoWindow.RES_Y))
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

                        print("Marks count: {}".format(i))

                        self.signal_markers_recorded.emit([self._point1[-1],self._point2[-1],self._point3[-1],self._point4[-1]])
                        self._init_marks = False


                    else:
                        print("Wrong number of marks. Expected 4. Detected {}".format(n_marks))
                                
                    
            

                elif self._track_marks:
                    print("Tracking marks1")
                    
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

                    self.signal_markers_coordinates.emit([self._point1[-1],self._point2[-1],self._point3[-1],self._point4[-1]])

                    
                #draw track of the markers
                for group in self._marks_groups:
                    lg = len(group)
                    if lg > 1:
                        print(group[lg-2])
                        cv2.line(self.__img_track, tuple(int(x) for x in group[lg-2]), tuple(int(x) for x in group[lg-1]), 150, 1)
            
            



    def __detectMarkers(self, image):
        """
        Detects markers in the given image and returns the resulting image with markers 
        drawn and the coordinates of the detected markers.
        """
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(image, 3)
        blur_strong = cv2.medianBlur(image, 71)
        #blur_strong = cv2.GaussianBlur(image, (71,71), 0)

        #blur_darker =  blur_strong.astype(np.float32) * 0.9
        #blur_darker = np.clip(blur_darker, 0, 255).astype(np.uint8)

        blur_inv = cv2.bitwise_not(blur)
        blur_strong_inv = cv2.bitwise_not(blur_strong)
        subtract_image = cv2.subtract(blur_inv, blur_strong_inv)

        #thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,15,2)
        #_, thresh = cv2.threshold(subtract_image, 250, 255, cv2.THRESH_BINARY)
        ret, thresh = cv2.threshold(subtract_image, 30, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations=3)
        
        opening = ~opening
        
        # Find circles 
        cnts = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        #print("contours {}".format(len(cnts[0])))
        #print("contours {}".format(len(cnts[1])))
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        
        
        
        marks_groups = []
        
        res_img = image
        
        for c in cnts:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c,True)
            if perimeter == 0:
                continue
            circularity = 4*math.pi*(area/(perimeter*perimeter))
           
            if 0.5 < circularity and area > 50 and area < 4000:
                '''
                ((x, y), r) = cv2.minEnclosingCircle(c)
                x = int (x)
                y = int (y)
                r = int (r)

                cv2.circle(res_img, (x, y), r, 120, 2) '''

                #Find center with image moments
                M = cv2.moments(c)

                #print(M)
    
                # Check if the moment "m00" is zero to avoid division by zero
                if M["m00"] != 0:
                    x = round(M["m10"] / M["m00"],3)
                    y = round(M["m01"] / M["m00"], 3)
                else:
                    x, y = 0, 0  # Assign some default value in case m00 is zero

                #print(x)

                cv2.drawContours(res_img, c, -1, 1, 1)
                cv2.circle(res_img, (int(x), int(y)), 1, 255, 2)
                
                
                  
                marks_groups.append((x,y))
        
        #(36, 255, 12)
        #img = cv2.drawContours(gray, cnts, 3, (0,255,0), 3)
        #print(type(marks_groups))
        
        return res_img, marks_groups
            
                    

class VideoWindow(QWidget):
    signal_update_roi = pyqtSignal(int, int, int, int)
    #start_tracking_signal = pyqtSignal(bool)  # Signal to start tracking marks
    
    RES_X = 640# int(1024/2)
    RES_Y = 480#int(768/2)
    

    def __init__(self, thread):
        

        """
        Initializes the video stream window and sets up the interface for displaying the video feed 
        and controlling the webcam. 
        """
        super().__init__()
        self.setWindowTitle("Video Stream")
        self._image_label = QLabel(self)
        self._image_label.resize(VideoWindow.RES_X, VideoWindow.RES_Y)
        self._layout = QVBoxLayout()
        self._layout.addWidget(self._image_label)
        self.setLayout(self._layout)

        self._button_stop = QPushButton('Stop Webcam', self)
        self._button_stop.clicked.connect(self.stop_webcam)
        self._layout.addWidget(self._button_stop)
        
        self._start_point = self._end_point = None
        
        self._draw_rectangle = False
         
        
        self.thread = thread
        self.thread.signal_change_pixmap.connect(self.update_image)
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
        self._draw_rectangle = True

        
        self.update()

    def mouseReleaseEvent(self, event):
        self._end_point = event.pos()
        
        print("start point {}".format(self._start_point))
        print("end point {}".format(self._end_point))
        self.signal_update_roi.emit(self._start_point.x(), self._start_point.y(), self._end_point.x(), self._end_point.y())
        self._draw_rectangle = False
        

    def stop_webcam(self):
        self.thread.stop()
        self.close()
        

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """
        Update the image displayed in the image label with the given cv_img.
        
        Args:
            cv_img: The OpenCV image to update the display with.
        """
        
        if self._start_point and self._end_point and self._draw_rectangle:
            start_p = (self._start_point.x(), self._start_point.y())
            end_p = (self._end_point.x(), self._end_point.y())
            cv2.rectangle(cv_img, start_p, end_p, 170, 3)
        
        ch = 1
        
        if (2 == len(cv_img.shape)):
            h, w = cv_img.shape
        else:
            h, w, _ = cv_img.shape
        
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8)
        p = convert_to_Qt_format.scaled(VideoWindow.RES_X, VideoWindow.RES_Y, Qt.AspectRatioMode.KeepAspectRatio)
        
 
        
        
    
        
        
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
