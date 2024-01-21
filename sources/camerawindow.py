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

from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
from PyQt6.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap
from datetime import datetime

from vimba import *


class markersDetection:
    
    '''
    class with methods to detect markers on the image
    
    returns
    image with highlighted markers
    list of tuples (x,y) with coordinates of each marker
    '''
    
    #fl_img to show that image with circles should be returned as a result, 
    #otherwise coordinates of markers
    def detectMarkers(self, image):
        """
        Detects markers in the given image and returns the resulting image with markers 
        drawn and the coordinates of the detected markers.
        """
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(image, 9)
        #thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,15,2)
        _, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel, iterations=3)
        
        opening = ~opening
        
        # Find circles 
        cnts = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
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
           
            if 0.7 < circularity and area > 50:
                ((x, y), r) = cv2.minEnclosingCircle(c)
                x = int (x)
                y = int (y)
                r = int (r)
                
                cv2.circle(res_img, (x, y), r, 120, 2)
                  
                marks_groups.append((x,y))
        
        #(36, 255, 12)
        #img = cv2.drawContours(gray, cnts, 3, (0,255,0), 3)
        #print(type(marks_groups))
        
        return res_img, marks_groups
     

class VideoThread(QThread):
    
    '''
    Class that constantly captures video from web camera and emits the signal to VideoWindow 
    to update the image.
    
    Has to flag to record first marks and to constantly track them.
    '''
    
    change_pixmap_signal = pyqtSignal(np.ndarray)

    
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
        self.marks_groups.append(self.group3)
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
        
        
    def send_command(self, string):
        
        if "record" == string:
            self._rec_marks = True
        elif "start" == string:
            self._track_marks = True
        elif "stop" == string:
            self.stop()
        


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
                            
                            
                            #allocate (x,y) in first empty sub(list)
                            i = 0
                            
                            for coord in coord_temp:
                 
                                if self.if_within_roi(coord) and i < len(self.marks_groups):
                                    self.marks_groups[i].append(coord)
                                    i += 1
                            
                            self._rec_marks = False
                            
                            #Get current date for filename
                            current_datetime = datetime.now()
                            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                            cv2.imwrite('Test_'+formatted_datetime+'_first_frame.jpg', img_cv)

                            print("Recorded marks:")
                            print(self.marks_groups)


                            
                            
                        elif self._track_marks:
                            
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
                        
                        
                #Get current date for filename
                current_datetime = datetime.now()
                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                cv2.imwrite('Test_'+formatted_datetime+'_tracks.jpg', img_track)
                cv2.imwrite('Test_'+formatted_datetime+'_last_frame.jpg', img_cv)
                
                self._track_marks = False
                #Record to the file
                self.writeDataToFile()
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
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.update_roi_signal.connect(self.thread.update_roi)
        
        self.init_command_thread()

    def init_command_thread(self):
        self.command_thread = QThread()
        self.command_thread.run = self.read_commands
        self.command_thread.start()

    def read_commands(self):
        while True:
            command = sys.stdin.readline().strip()  # Read a line from stdin
            
            if "stop" == command:
                self.stop_webcam()
            
            #self.received_command.emit(command)  # Emit the signal
            
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
        # This method is called when the window is closed.
        self.__terminate_application()
        self.stop_webcam()

    def __terminate_application(self):
        pass
        
    def track_marks_clicked(self):
        """
        A function to track when marks are clicked, emitting a signal.
        """
        self._track_marks_signal.emit()  # Emit the signal


if __name__ == '__main__':
    print("Started")
    app = QApplication(sys.argv)
    video_thread = VideoThread()
    video_window = VideoWindow(video_thread)
    video_thread.start()
    video_window.show()
    sys.exit(app.exec())
