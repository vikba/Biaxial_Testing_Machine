# -*- coding: utf-8 -*-
"""
Created on Sun Jan  7 09:03:14 2024

@author: V
"""

import sys
import cv2
#import math
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
    
    change_pixmap_signal = pyqtSignal(np.ndarray)
    signal_marks_recorded = pyqtSignal(list)

    
    def __init__(self):
        super().__init__()
     
        self._track_marks = False  # Initialize the flag
        self._rec_marks = False  # Initialize the flag
        
        self.__initVariables()
        
        self._execute = True
            
        self._roi_x1 = 0
        self._roi_y1 = 0
        
        self._roi_x2 = 1024
        self._roi_y2 = 768

    def __initVariables(self):
        """
        Initialize variables to store tracks of marks and groups.
        """

        #datastructures to store tracks of marks
        self.marks_groups = []
        self.group1 = []
        self.group2 = []
        self.group3 = []
        self.group4 = []
        self.group5 = []
        
        self.marks_groups.append(self.group1)
        self.marks_groups.append(self.group2)
        #self.marks_groups.append(self.group3)
        self.marks_groups.append(self.group4)
        self.marks_groups.append(self.group5)

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
        
        self._rec_marks = True
        
    def if_within_roi (self, p):
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
        


    def startStopTracking(self, flag):
        """
        This function starts or stops tracking based on the value of the flag parameter.
        It takes in the flag parameter, which is a boolean to indicate whether to start or stop tracking.
        There are no return types specified for this function.
        """
        if flag:
            print("Start tracking")
            self._track_marks = True  # Change the flag
            self._start_time = time.perf_counter()
            self._current_time = 0
        
        else:
            self._execute = False
            self.stop()
    
    
    def run(self):
        """
        The run function controls the execution of the main camera loop. 
        It initializes the camera and sets its parameters, then enters a loop where it 
        continuously captures frames from the camera. 
        It processes each frame to detect markers and record their positions. 
        It also calculates strain and saves the tracked marks to images. 
        """
        #self.cap = cv2.VideoCapture(0)
        
        img_track = np.zeros((768, 1024, 1), dtype=np.uint8)
        img_track.fill(0)
        self._time = []
        
        with Vimba.get_instance () as vimba:
            cams = vimba.get_all_cameras ()
            with cams [0] as cam:
                
                cam.Gain.set(10)
                cam.ExposureTime.set(1000)
        
                while self._execute:
                    
                    
                    t = time.perf_counter()
                    time.sleep(0.1)
                    
                    
                    
                    frame = cam.get_frame ()
                    
                    if frame:
                        
                        img_cv = frame.as_opencv_image()
                        
                        #print("Time 1 {}".format(time.perf_counter()-t))
                        
                        #detect markers
                        img, coord_temp = markersDetection().detectMarkers(img_cv) #detection of Markers
                        
                        #print("Time 2 {}".format(time.perf_counter()-t))
                        #img = img_cv
                        #coord_temp = []
                        #print(coord_temp)
                        
                        #draw track of the markers
                        for group in self.marks_groups:
                            lg = len(group)
                            if lg > 1:
                                cv2.line(img_track, group[lg-2], group[lg-1], 150, 1)
                            
                           
                        # True only one time to record initial position of the marks
                        if self._rec_marks:

                            self.__initVariables()
                            
                            print("Start recording")
                            print("Point 1 {},  {}".format(self._roi_x1, self._roi_y1))
                            print("Point 2 {},  {}".format(self._roi_x2, self._roi_y2))
                            
                            if len(coord_temp) == 4:
                                #allocate (x,y) in first empty sub(list)
                                i = 0
                                for coord in coord_temp:
                    
                                    if self.if_within_roi(coord) and i < len(self.marks_groups):
                                        self.marks_groups[i].append(coord)
                                        i += 1

                                #Get current date for filename
                                current_datetime = datetime.now()
                                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                                cv2.imwrite('Test_'+formatted_datetime+'_first_frame.jpg', img_cv)

                                self.signal_marks_recorded.emit(self.marks_groups)
                                #print("Recorded marks:")
                                #print(self.marks_groups)
                                self.stop()

                            else:
                                print("Wrong number of marks. Expected 4. Detected {}".format(len(coord_temp)))
                                warning_box = QMessageBox()
                                warning_box.setIcon(QMessageBox.Icon.Warning)
                                warning_box.setWindowTitle("Warning")
                                warning_box.setText("Wrong number of marks. Expected 4. Detected {}".format(len(coord_temp)))
                                warning_box.exec()
                                
                            self._rec_marks = False
                            
                            


                            
                            
                        elif False: #self._track_marks:
                            
                            self._current_time = time.perf_counter() - self._start_time
                            self._time.append(round(self._current_time, 5))
            
                            l = len(self.marks_groups)
                            
                            #element - pair of (x,y) coordinates of a mark
                            #distribute marks in the groups
                            for element in coord_temp: 
                                #print("(x,y) {}".format(element))
                                min_dist = 2000
                                group = []
                                #For each element we are looking for a marks group with a lowest distance
                                for i in range (0,l):
                                    gr = self.marks_groups[i]
                                    if len(gr) > 0:
                                        last = gr[len(gr) - 1]
                                        dist = math.dist(element, last)
                                        #print("dist {} and {} = {}".format(element, last, dist))
                                        if min_dist > dist and dist < 70:
                                            group = gr
                                            min_dist = dist
                                
                                group.append(element)


                            #calculate strain
                            #along horizontal axis - upper and lower groups]
                            #the algorithm find contours arranges points along y axis of an image
                            eps_ab = (math.dist(self.group1[-1],self.group2[-1])-math.dist(self.group1[0],self.group2[0]))/math.dist(self.group1[-1],self.group2[-1])
                            eps_cd = (math.dist(self.group4[-1],self.group5[-1])-math.dist(self.group4[0],self.group5[0]))/math.dist(self.group4[-1],self.group5[-1])
                            lambda_1 = (2 + eps_ab + eps_cd)/2
                            E11 = (lambda_1*lambda_1-1)/2

                            #check orientation of the points
                            if (self.group1[0][1]-self.group4[0][1] < self.group1[0][1]-self.group5[0][1]):
                                eps_ac = (math.dist(self.group1[-1],self.group4[-1])-math.dist(self.group1[0],self.group4[0]))/math.dist(self.group1[-1],self.group4[-1])
                                eps_bd = (math.dist(self.group2[-1],self.group5[-1])-math.dist(self.group2[0],self.group5[0]))/math.dist(self.group2[-1],self.group5[-1])
                            else:
                                eps_ac = (math.dist(self.group1[-1],self.group5[-1])-math.dist(self.group1[0],self.group5[0]))/math.dist(self.group1[-1],self.group5[-1])
                                eps_bd = (math.dist(self.group2[-1],self.group4[-1])-math.dist(self.group2[0],self.group4[0]))/math.dist(self.group2[-1],self.group4[-1])

                            lambda_2 = (2 + eps_ac + eps_bd)/2
                            E22 = (lambda_2*lambda_2-1)/2

                            self._E11.append(E11)
                            self._E22.append(E22)
                            
                            print("E11 {} E22 {}".format(E11, E22))

                            #print(self.marks_groups)
                        
                        #append marks coordinates to the tracks to that they have the lowest distance 
                        #lowest distance - they are from this track
                        
                   
                        img = cv2.addWeighted(img_track, 1, img, 1, 0)

                        
                        #rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                       
                        self.change_pixmap_signal.emit(img)
                        
                        #print("One frame {}".format(time.perf_counter()-t))
                        
                        
                
                self._track_marks = False
                #Record to the file
                #self.writeDataToFile()
                self.quit()
            
    def writeDataToFile(self):
        # Combine the lists
        combined_lists = zip(self._time, self._E11, self._E22, self.group1, self.group2, self.group3, self.group4, self.group5)
        
        #Get current date for filename
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        
        
        # Write to CSV file
        with open('Test_'+formatted_datetime+'_markers.csv', 'w', newline='') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(["Time", "E11", "E22", "Marker_1", "Marker_2", "Marker_3", "Marker_4", "Marker_5"])  # Header row, if needed
            for row in combined_lists:
                writer.writerow(row)
                
                
                    

class VideoWindow(QWidget):
    update_roi_signal = pyqtSignal(int, int, int, int)
    #start_tracking_signal = pyqtSignal(bool)  # Signal to start tracking marks
    
    
    def __init__(self, thread):
        """
        Initializes the video stream window and sets up the interface for displaying the video feed 
        and controlling the webcam. 
        """
        super().__init__()
        self.setWindowTitle("Video Stream")
        self.disply_width = 1024
        self.display_height = 768
        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_label)
        self.setLayout(self.layout)

        self.button_stop = QPushButton('Stop Webcam', self)
        self.button_stop.clicked.connect(self.stop_webcam)
        self.layout.addWidget(self.button_stop)
        
        '''
        self.button_tr_marks = QPushButton('Start tacking', self)
        self.button_tr_marks.clicked.connect(self._track_marks_clicked)
        self.layout.addWidget(self.button_tr_marks)
        '''
        self.start_point = self.end_point = None
        
        self._track_marks = False
         
        
        self.thread = thread
        #self.thread.change_pixmap_signal.connect(self.update_image)
        self.update_roi_signal.connect(self.thread.update_roi)
        
            
    def mousePressEvent(self, event):
        """
        Handle the mouse press event and update the start point, end point, and trigger an update.
        """
        self.start_point = event.pos()
        self.end_point = None
        self.update()

    def mouseMoveEvent(self, event):
        self.end_point = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        self.end_point = event.pos()
        
        print("start point {}".format(self.start_point))
        print("end point {}".format(self.end_point))
        self.update_roi_signal.emit(self.start_point.x(), self.start_point.y(), self.end_point.x(), self.end_point.y())

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
        
        if self.start_point and self.end_point:
            start_p = (self.start_point.x(), self.start_point.y())
            end_p = (self.end_point.x(), self.end_point.y())
            if not self._track_marks:
                cv2.rectangle(cv_img, start_p, end_p, 170, 3)
        
        ch = 1
        
        if (2 == len(cv_img.shape)):
            h, w = cv_img.shape
        else:
            h, w, _ = cv_img.shape
        
        
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format.Format_Grayscale8)
        p = convert_to_Qt_format.scaled(1024, 768, Qt.AspectRatioMode.KeepAspectRatio)
        
 
        
        '''
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
        '''
        
        qt_img = QPixmap.fromImage(p)
        self.image_label.setPixmap(qt_img)
        

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
