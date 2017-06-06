import numpy as np
from collections import deque
import argparse
import imutils
import cv2
from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.calibration import StereoCalibration
from picamera.array import PiRGBArray
from picamera import PiCamera
from multiprocessing import Process
from time import sleep
import RPi.GPIO as gp

def setupBoard():
    """Enables the correct GPIO pins
    to allow for the Pi to interface
    with up to 4 cameras using the
    Arducam Multi Camera Adapter
    Module."""
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
    """Enables the correct GPIO pins to
    enable the correct camera using the
    Arducam Multi Camera Adapter
    Module."""
    
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
res = (320, 240)
camera.resolution = res
camera.framerate = 5
##camera.exposure_mode = 'off'
rawCapture = PiRGBArray(camera, size=res)
calibration = StereoCalibration(input_folder="/home/pi/Desktop/programs2017/PiCAM PHOTOS/calibfiles")
block_matcher = StereoBM() # from stereovision.blockmatchers

block_matcher.search_range = 64
block_matcher.bm_preset = 0
block_matcher.window_size = 27




setupBoard()
sleep(0.5) # enable camera to start up

counter = 0
try: 
    while True:
        counter += 1

        cameras("C")
        if counter == 7:
            camera.exposure_mode = 'off'
            camera.shutter_speed = (camera.exposure_speed)
            camera.awb_mode = 'fluorescent'
        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameC = rawCapture.array

##        cv2.imshow("frameC", frameC)
        rawCapture.truncate(0)

        sleep(0.03)

        cameras("A")
        if counter == 7:
            camera.exposure_mode = 'off'
            camera.shutter_speed = (camera.exposure_speed)
            camera.awb_mode = 'fluorescent'
            print("auto exposure off")

        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameA = rawCapture.array
        
##        cv2.imshow("frameA", frameA)
        rawCapture.truncate(0)
    
        rectified_pair = calibration.rectify((frameA,frameC))

        disparity = block_matcher.get_disparity(rectified_pair)

        disparity = disparity / disparity.max()

        #disparity2 = cv2.cvtColor(disparity, cv2.COLOR_GRAY2BGR)
        #shapeMask = cv2.inRange(disparity, 0.9, 1.0)
        _, shapeMask = cv2.threshold(disparity, 0.9, 1.0, cv2.THRESH_BINARY)
        
##        cv2.imshow("disparity", disparity)
##        cv2.imshow("disparity2", shapeMask)

        print(shapeMask.shape)
        #print(shapeMask)

        disparity3 = shapeMask.copy()
        disparity3 = disparity3 * 255.0
        disparity3 = disparity3.astype(int)

        #print(frameA)
        #print(disparity3)

        #disparity3 = cv2.cvtColor(disparity3, cv2.COLOR_BGR2GRAY);

        #disparity3[disparity3 != 0.0] = 1.0

        if counter == 5:
            
            print(shapeMask)
            print(shapeMask.max())
            print(disparity)
            print(disparity.max())
        disparity3 = cv2.convertScaleAbs(disparity3)
        contours, _ = cv2.findContours(disparity3, 1, 2)

        disparity3 = cv2.cvtColor(disparity3, cv2.COLOR_GRAY2RGB)

##        print(contours)
        for cnt in contours:
            #[x,y,w,h] = cv2.boundingRect(cnt)
            #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)

            # store 4 corners to box
            
            box = np.int0(box)
##            yMax = 0
##            yMin = 0
            middle = False
            for point in box:
                if (point[1] > 80 and point[1] < 160):
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                    middle = True
                    break
##                elif point[1]>160 and point[1]>yMax:
##                   yMax = point[1]
##                elif point[1]<80 and point[1]<yMin:
##                    yMin = point[1]
##            if yMin != 0 && yMax !=00

            if not middle:
                yMin = max(box, key=lambda x:x[1])
                yMin = yMin[1]
                yMax = max(box, key=lambda x:x[1])
                yMax = yMax[1]


                if yMax > 160 and yMin < 80:
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                
            # blue rectangle
            # print(box)

##        cv2.imshow("disparity3", disparity)
##        cv2.imshow("a",frameA)
##        cv2.imshow("c",frameC)
        cv2.line(disparity, (0,80), (320,80), 1, 8, 0)
        cv2.line(disparity, (0,160), (320,160), 1, 8, 0)
        cv2.imshow("disparity w/ boxes", disparity)

        key = cv2.waitKey(1)


        
except KeyboardInterrupt:
    print(disparity)
    print(disparity.max())
    print(shapeMask)
    print(shapeMask.max())
    pass
    
cv2.destroyAllWindows()


