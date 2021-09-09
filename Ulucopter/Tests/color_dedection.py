import re
import cv2,keyboard
import numpy as np


def kirmiziAlgila():
    camera = cv2.VideoCapture(0)
    while True:
        _,frame = camera.read()       
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        low_red = np.array([0, 174, 0])
        high_red = np.array([5, 255, 255])
        red_mask = cv2.inRange(hsv, low_red, high_red)
        dilated_redmask = cv2.dilate(red_mask, kernal)
        blured_redmask = cv2.GaussianBlur(dilated_redmask, (5,5), 4/6)
        contours_red, hierarchy = cv2.findContours(blured_redmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        i=0
        for pic, contour in enumerate(contours_red):
            area = cv2.contourArea(contour)
            if area > 3000 :
                i+=1
                if i == 2:             
                    break
            return True
        else:
            return False
        camera.release()
    
def maviAlgila():
    camera = cv2.VideoCapture(0)
    while True:
        _,frame = camera.read()       
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        kernal = np.ones((5,5),"uint8")
        i=0
        low_blue = np.array([100, 135, 40])
        high_blue = np.array([115, 255, 255])
        blue_mask = cv2.inRange(hsv, low_blue, high_blue)
        dilated_bluemask = cv2.dilate(blue_mask, kernal)
        blured_bluemask = cv2.GaussianBlur(dilated_bluemask, (5,5), 4/6)
        contours_blue, hierarchy = cv2.findContours(blured_bluemask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours_blue):
            area = cv2.contourArea(contour)
            if area > 3000 :
                i+=1
                if i == 2:
                    break
            return True
                    
        else:
            return False
        camera.release()     
       
        

