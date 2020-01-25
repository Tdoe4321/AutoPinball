import cv2
import numpy as np


def mouseRGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = frame[y,x,0]
        colorsG = frame[y,x,1]
        colorsR = frame[y,x,2]
        colors = frame[y,x]
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        hsv = np.uint8([[[colorsB,colorsG,colorsR ]]])
        hsv = cv2.cvtColor(hsv,cv2.COLOR_BGR2HSV)
        print("BRG Format: ",colors)
        print("HSV Format: ", hsv)
        print("Coordinates of pixel: X: ",x,"Y: ",y)


cv2.namedWindow('mouseRGB',cv2.WINDOW_NORMAL)
cv2.setMouseCallback('mouseRGB',mouseRGB)
cv2.resizeWindow('mouseRGB',800,800)

capture = cv2.VideoCapture(2)

while(True):

    ret, frame = capture.read()

    cv2.imshow('mouseRGB', frame)

    if cv2.waitKey(1) == 27:
        break

capture.release()
cv2.destroyAllWindows()
