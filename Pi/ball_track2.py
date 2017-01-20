from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import serial
import cv2
import numpy as np;

ser = serial.Serial('/dev/ttyACM0', 9600)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.resolution = (640, 480)
camera.resolution = (480, 320)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480, 320))

#lower = (20,100,150)
#upper = (95,255,255)

lower = (40,230,40)
upper = (70,255,250)
def trans(n):
        s = str(n)
        for i in range(0,4 - len(s)):
                s = '0' + s
        return s

# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

	blurred = cv2.GaussianBlur(image, (15,15),0)
        # Make the mask
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x,y), radius) = cv2.minEnclosingCircle(c)
                M= cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 20:
                        cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image, center, 5, (0,0, 255), -1)
                        print center
                        #ser.write(str(center))
                        #str(int(M["m10"] / M["m00"])) + "," + str(int(M["m01"] / M["m00"]))

 
	# show the frame
	cv2.imshow("Frame", image)
	#cv2.imshow("Frame2", mask)
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

##while True:
##    rval, frame =cam.read()
##    blurred = cv2.GaussianBlur(frame, (11,11),0)
##    # Make the mask
##    #hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
##    mask = cv2.inRange(rgb, lower, upper)
##    mask = cv2.erode(mask, None, iterations=2)
##    mask = cv2.dilate(mask, None, iterations=2)
##    cv2.imshow("Keypoints2", mask)
##    cv2.waitKey(0)
##        
