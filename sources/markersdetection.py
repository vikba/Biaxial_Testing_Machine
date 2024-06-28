import cv2
import math
import numpy as np

class markersDetection:
    '''
    class with methods to detect markers on the image
    '''
    
    #fl_img to show that image with circles should be returned as a result, 
    #otherwise coordinates of markers
    def detectMarkers(self, image):
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