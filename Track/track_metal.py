import cv2
from matplotlib import pyplot as plt
import numpy as np

camera = cv2.VideoCapture(2)

while True:    
    ret, img = camera.read()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, .5, 50, param1=10, param2=90, minRadius=0, maxRadius=100)
    if circles is not None:
        for x, y, r in circles[0]:
            cv2.circle(img, (x,y), r, (0, 255, 255), 2)

    # show the frame to our screen
    cv2.imshow("Frame", img)
    key = cv2.waitKey(1) & 0xFF
 
	# if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
