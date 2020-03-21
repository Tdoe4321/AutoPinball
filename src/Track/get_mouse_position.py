#!/usr/bin/env python

import cv2
import numpy as np

ix,iy = -1,-1
# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),10,(0,0,0),3)
        ix,iy = x,y
        print("["+str(ix) + "," +  str(iy) + "],")

# Create a black image, a window and bind the function to window
camera = cv2.VideoCapture(0)
ret, img = camera.read()
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(20) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()
