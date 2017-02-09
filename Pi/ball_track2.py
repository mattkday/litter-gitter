from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import time
import RPi.GPIO as GPIO
import smbus
import cv2
import numpy as np;
#start the i2c
bus = smbus.SMBus(1)
# setup the GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Arduino address
address = 0x04

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1024/4, 768/4)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(1024/4, 768/4))

##lower = (10,230,40)
##upper = (120,255,250)

# define the color range
lower = (50,130,50)
upper = (100,255,250)

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab current frame from camera
    image = frame.array
    # blur image
    blurred = cv2.GaussianBlur(image, (15,15),0)
    # Make the mask
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    #find the object
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
            # send the points if the arduino wants them
            if GPIO.input(10):
                # send the object's points to the Arduino for further use
                try:
                    bus.write_byte_data(address, 0x01, int(M["m10"] / M["m00"]))
                    time.sleep(0.01)
                    bus.write_byte_data(address, 0x02, int(M["m01"] / M["m00"]))
                except IOError:
                    sys.stderr.write("*** error: sending points ***\n")
            else:
                sys.stderr.write("* Points were not sent *\n")


    # show the frame
    cv2.imshow("Frame", image)
    #cv2.imshow("Frame2", mask)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        GPIO.cleanup()
        break
