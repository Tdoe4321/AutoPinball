# Computer Vision packages
import cv2
import numpy as np

# Wait and time diff
import time

# List for points
from collections import deque

# Smooting algorithm
from scipy import signal

# ROS
import rospy
from pinball_messages.msg import flip_flipper

def calculate_thresh(first_frame, current_frame):
    frame_delta = cv2.absdiff(first_frame, current_frame)
    frame_delta = cv2.GaussianBlur(frame_delta, (7,7), 0)
    #frame_delta = cv2.filter2D(frame_delta,-1,kernel)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
    #return thresh
    #thresh = cv2.erode(thresh, None, iterations=9)
    #img = cv2.adaptiveThreshold(frame_delta,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)

    thresh = cv2.dilate(thresh, None, iterations=2)
    #return thresh
    cnts, result = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return cnts

def draw_line(img, pts):
    # loop over the set of tracked points
    for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore them
		if pts[i - 1] is None or pts[i] is None:
			continue
		# otherwise, compute the thickness of the line and draw the connecting lines
		thickness = int(np.sqrt(line_length / float(i + 1)) * 2.5)
		cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)
        
# Camera Object
camera = cv2.VideoCapture(2)

# Convolution filter
kernel = np.matrix('-1 -1 -1; -1 10 -1; -1 -1 -1')

# Drawing line
line_length = 64
pts = deque(maxlen=line_length)
x_pts = deque(maxlen=line_length)
y_pts = deque(maxlen=line_length)
smooth_pts = deque(maxlen=line_length)

# Let the camera startup and autofocus
camera.read()
time.sleep(1)

# Capture the first frame to compare with in the binary difference
ret, first_frame = camera.read()
first_frame = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

# Trhesholds for size of the ball
THRESH_MAX = 1500
THRESH_MIN = 1000

# Current coordinates for the ball
ball_x = None
ball_y = None

# ROS objects
publish_flipper = rospy.Publisher('internal_flip_flipper', flip_flipper, queue_size=10)

if __name__ == "__main__":
    # Startup ROS node
    rospy.init_node("Tracking_Ball")
    time.sleep(1)
    
    while not rospy.is_shutdown():    
        # Grab current image
        ret, img = camera.read()
    
        # Make it grayscale
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ''' HOUGH CIRCLES TECHNIQUE
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 0.5, img.shape[0]/4, param1=220, param2=20, minRadius=0, maxRadius=100)
        # param 2 = ~20, min = 10, max = 25
        if circles is not None:
            for x, y, r in circles[0]:
                cv2.circle(img, (x,y), r, (0, 255, 255), 2)
        '''
    
        # TO SHOW JUST TRESHHOLD STUFF
        #img = calculate_thresh(first_frame, img)

        # Get a list of contours
        cnts = calculate_thresh(first_frame, img)
        if cnts is not None:
            # Draw them on the image
            cv2.drawContours(img, cnts, -1, (0, 255, 0), 3) 
            for c in cnts:
                # if the contour is too small or too big, ignore it
                if cv2.contourArea(c) < THRESH_MAX and cv2.contourArea(c) > THRESH_MIN:
                    (x, y, w, h) = cv2.boundingRect(c)
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    ball_x = x+w/2
                    ball_y = y+h/2
                    cv2.circle(img, (ball_x, ball_y), 20, (0,0,255), -1)
                    pts.appendleft((ball_x,ball_y))
                    print("Contour size: " + str(cv2.contourArea(c)))
                    '''
                    x_pts.appendleft(ball_x)
                    y_pts.appendleft(ball_y)
                    if len(x_pts) > 9 and len(y_pts) > 9:
                        smooth_pts = signal.savgol_filter(x_pts, 9, 3),signal.savgol_filter(y_pts, 9, 3)
                    '''

                else:
                    # If we didn't get one for the ball, set to None
                    ball_x = None
                    ball_y = None

        ''' LINE USING SMOOTHED POINTS
        if len(x_pts) > 9 and len(y_pts) > 9:
            # loop over the set of tracked points
            for i in range(1, len(smooth_pts)):
	        	# if either of the tracked points are None, ignore
	        	# them
	        	if smooth_pts[i - 1] is None or smooth_pts[i] is None:
	        		continue
	        	# otherwise, compute the thickness of the line and
	        	# draw the connecting lines
	        	thickness = int(np.sqrt(line_length / float(i + 1)) * 2.5)
	        	cv2.line(img, (int(smooth_pts[0][i - 1]),int(smooth_pts[1][i-1])), (int(smooth_pts[0][i]),int(smooth_pts[1][i])), (0, 0, 255), thickness)
        '''

        # Draw connecting history line
        draw_line(img, pts)            

        # show the frame to our screen
        cv2.imshow("Frame", img)
        key = cv2.waitKey(1) & 0xFF
 
	    # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
