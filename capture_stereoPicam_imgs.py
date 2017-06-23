import numpy as np
import cv2
from matplotlib import pyplot as plt
import cv2
from stereovision.calibration import StereoCalibration
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep
import Image
import RPi.GPIO as gp
import os

def setupBoard():
    '''Enables the correct GPIO pins
    to allow for the Pi to interface
    with up to 4 cameras using the
    Arducam Multi Camera Adapter
    Module.'''
    gp.setwarnings(False)
    gp.setmode(gp.BOARD)

    gp.setup(7, gp.OUT)
    gp.setup(11, gp.OUT)
    gp.setup(12, gp.OUT)
     
    gp.setup(15, gp.OUT)
    gp.setup(16, gp.OUT)
    gp.setup(21, gp.OUT)
    gp.setup(22, gp.OUT)

    gp.output(11, True)
    gp.output(12, True)
    gp.output(15, True)
    gp.output(16, True)
    gp.output(21, True)
    gp.output(22, True)

def cameras(cam):
    '''Enables the correct GPIO pins to
    enable the correct camera using the
    Arducam Multi Camera Adapter
    Module.'''
    
    if cam == "A":
        gp.output(7, False)
        gp.output(11, False)
        gp.output(12, True)
    elif cam == "B":
        gp.output(7, True)
        gp.output(11, False)
        gp.output(12, True)
    elif cam == "C":
        gp.output(7, False)
        gp.output(11, True)
        gp.output(12, False)
    elif cam == "D":
        gp.output(7, True)
        gp.output(11, True)
        gp.output(12, False)
        
    else: raise ValueError, "Need to input A, B, C, or D for cameras."

def finished():
    '''Finished with all the cameras.'''
    
    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)

camera = PiCamera()
res = (640, 480)
camera.resolution = res
camera.framerate = 90
rawCapture = PiRGBArray(camera, size=res)

setupBoard()
sleep(0.5) # enable camera to start up

counter = 0
counter2 = 0
try: 
    while True:
        cameras("C")
        if counter == 7:
            camera.exposure_mode = 'off'
            camera.shutter_speed = (camera.exposure_speed)
            camera.awb_mode = 'fluorescent'
            print("auto exposure off")
        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameC = rawCapture.array
##        frameC = cv2.cvtColor(frameC, cv2.COLOR_BGR2GRAY)
##        rows, cols = frameC.shape
##        M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
##        frameC = cv2.warpAffine(frameC,M,(cols,rows))
        # display the image on screen and wait for a keypress
        cv2.imshow("frameC", frameC)
    ##    key = cv2.waitKey(1)
        rawCapture.truncate(0)

        sleep(0.03)

        cameras("A")

        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameA = rawCapture.array
##        frameA = cv2.cvtColor(frameA, cv2.COLOR_BGR2GRAY)


        # display the image on screen and wait for a keypress
        cv2.imshow("frameA", frameA)
    ##    key = cv2.waitKey(1)
        rawCapture.truncate(0)

        key = cv2.waitKey(10)


        if counter%5 == 0:
            counter2 += 1
            if(counter2 <10):
                
                cv2.imwrite("left_0"+str(counter2)+".ppm", frameA)
                cv2.imwrite("right_0"+str(counter2)+".ppm", frameC)
            else:
                cv2.imwrite("left_"+str(counter2)+".ppm", frameA)
                cv2.imwrite("right_"+str(counter2)+".ppm", frameC)
            print("Saved image "+ str(counter2))
        counter += 1

        
except KeyboardInterrupt:
        cv2.imwrite("leftimg.jpg", frameA)
        cv2.imwrite("rightimg.jpg", frameC)
        pass
##vcL.release()
##vcR.release()
cv2.destroyAllWindows()
