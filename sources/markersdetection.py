import cv2
import math
import numpy as np

class markersDetection:
    '''
    class with methods to detect markers on the image
    '''
    
    #fl_img to show that image with circles should be returned as a result, 
    #otherwise coordinates of markers
    