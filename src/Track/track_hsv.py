#!/usr/bin/env python

# import the necessary packages
import numpy as np
import cv2
import time

# define the lower and upper boundaries
hsvLower = (15, 200, 100)
hsvUpper = (45, 270, 220)

camera = cv2.VideoCapture(2)

time.sleep(1.0)

# keep looping
while True:
    # grab the current frame
    ret, frame = camera.read()
    
    blurred = cv2.GaussianBlur(frame, (15, 15), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    #mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)
    
    cnts, hie = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) >= 2:
        # Sort and grab the two biggest
        cnts.sort(key=cv2.contourArea, reverse=True)
        big_cnt = cnts[0]
        big_cnt_2 = cnts[1]

        # Draw the first
        rect = cv2.minAreaRect(big_cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

        # Draw the second
        rect = cv2.minAreaRect(big_cnt_2)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
    
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    cv2.imshow("mask", mask)
    key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break


# close all windows
cv2.destroyAllWindows()
