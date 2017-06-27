import numpy as np
from matplotlib import pyplot as plt
import cv2
from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.calibration import StereoCalibration
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep, time
import Image
import RPi.GPIO as gp
import os
import ps_drone
import math


auto = True

def avoidBack():
##    drone.moveBackward()
##    print("moving back")
##    sleep(.8)
##    print("stopped")
##    drone.stop()
##    sleep(1)

    
##    print("turning")
##    drone.turnAngle(45, 1)
##    sleep(2)
##    drone.turnAngle(30, 1)
##    sleep(2)
    
##    drone.stop()
##    drone.turnAngle(30, 5)
##    drone.stop()
##    drone.turnAngle(30, 5)
##    print("moving back")
##    drone.moveBackward()

    print("landing")
##    drone.land()
  
def maxRatio(box, ratio):
    """Determine length/width ratio of bounding box.
    Returns TRUE if ratio is smaller than maxRatio.
    Else return FALSE."""
    dist1 = math.sqrt((box[0][0]-box[1][0])**2 + (box[0][1]-box[1][1])**2)
    dist2 = math.sqrt((box[0][0]-box[3][0])**2 + (box[0][1]-box[3][1])**2)
    
    if dist1/(dist2*1.0) > (ratio*1.0) or dist1/(dist2*1.0) < 1/(1.0*ratio):
        return False
    return True

def keyboardDrone(drone):
    global auto
    key = drone.getKey()
    if key == " ":      drone.takeoff()
    elif key == "l":    drone.land()
        #if drone.NavData["demo"][0][2] and not drone.NavData["demo"][0][3]:	drone.takeoff()
        #else:																drone.land()
    elif key == "0":	drone.hover()
    elif key == "w":	drone.moveForward()
    elif key == "s":	drone.moveBackward()
    elif key == "a":	drone.moveLeft()
    elif key == "d":	drone.moveRight()
    elif key == "q":	drone.turnLeft()
    elif key == "e":	drone.turnRight()
    elif key == "7":	drone.turnAngle(-10,1)
    elif key == "9":	drone.turnAngle( 10,1)
    elif key == "4":	drone.turnAngle(-45,1)
    elif key == "6":	drone.turnAngle( 45,1)
    elif key == "1":	drone.turnAngle(-90,1)
    elif key == "3":	drone.turnAngle( 90,1)
    elif key == "8":	drone.moveUp()
    elif key == "2":	drone.moveDown()
    elif key == "*":	drone.doggyHop()
    elif key == "+":	drone.doggyNod()
    elif key == "-":	drone.doggyWag()
    elif key == "o":    auto = not auto
        
##drone = ps_drone.Drone()
##drone.startup()
##drone.setSpeed(0.1)


calibration = StereoCalibration(input_folder = "/home/pi/Desktop/programs2017/UAV-Stereo-Vision-Webcams/USBcamCalib8/calibFiles")
block_matcher = StereoBM() # from stereovision.blockmatchers
block_matcher.search_range = 48
block_matcher.bm_preset = 0
block_matcher.window_size = 31
##minBoxArea = 2000

minBoxArea = 2000
maxLengthWidthRatio = 3
height = 480
width = 640

vcL = cv2.VideoCapture(0)
vcR = cv2.VideoCapture(1)
vcL.set(3,width)
vcL.set(4,height)
vcR.set(3,width)
vcR.set(4,height)
vcL.set(5,5)
vcR.set(5,5)
vcL.set(21,1)
vcR.set(21,1)

sleep(0.5) # enable camera to start up

counter = 0
counter2 = 0

global imgL
global imgR
global rectified_pair
global disparity
global shapeMask
global disparityProcessed
global temp

print("initializing globals")

counter = 0
rvalL, imgL = vcL.read()
rvalR, imgR = vcR.read()
while imgL == None or imgR == None:
    rvalL, imgL = vcL.read()
    rvalR, imgR = vcR.read()
temp = imgL
key = cv2.waitKey(10)
rectified_pair = calibration.rectify((imgL, imgR))
disparity = block_matcher.get_disparity(rectified_pair)
_, shapeMask = cv2.threshold(disparity, 0.7, 1.0, cv2.THRESH_BINARY)
disparityProcessed = shapeMask.copy()

print("cameras ready")

# write frame time to a txt file
timeData = open("timeData.txt", 'w')
# clear old data
timeData.truncate()
##sleep(60)
##drone.takeoff()

sample = 1.0

try: 
    while True:
##        keyboardDrone(drone)
        counter += 1
        # begin frame calculation
        startTime = time()
        
        rvalL, imgL = vcL.read()
        rvalR, imgR = vcR.read()
        
##        imgL = cv2.resize(imgL, (0,0), fx=1.0/sample, fy=1.0/sample)
        
##        frameC = cv2.cvtColor(frameC, cv2.COLOR_BGR2GRAY)
        if(counter == 5):
            realheight, realwidth = imgL.shape[:2]
            print("Real width: " + str(realwidth))
            print("Real height: " + str(realheight))

##        cv2.imshow("imgL -- left", imgL)

        if counter % 10 == 0:

    ##        imgR = cv2.resize(imgR, (0,0), fx=1.0/sample, fy=1.0/sample)
            

            # display the image on screen and wait for a keypress
    ##        cv2.imshow("imgR -- right", imgR)
        ##    key = cv2.waitKey(1)

            key = cv2.waitKey(10)

            rectified_pair = calibration.rectify((imgL, imgR))
            disparity = block_matcher.get_disparity(rectified_pair)
            disparity = disparity / disparity.max()
            

            # normalize disparity in [0, 1]

            _, shapeMask = cv2.threshold(disparity, 0.7, 1.0, cv2.THRESH_BINARY)
            
    ##        shapeMask = cv2.morphologyEx(shapeMask, cv2.MORPH_CLOSE,kernel)
            disparityProcessed = shapeMask.copy()
            disparityProcessed = disparityProcessed * 255.0
            disparityProcessed = disparityProcessed.astype(int)
            # bring into 255 range

            disparityProcessed = cv2.convertScaleAbs(disparityProcessed)
            
            contours, _ = cv2.findContours(disparityProcessed, 1, 2)
            # find contours from thresholded disparity map

            disparityProcessed = cv2.cvtColor(disparityProcessed, cv2.COLOR_GRAY2RGB)
    ##        cv2.imshow("disparityProcessed", disparityProcessed)
    ##        cv2.imshow("left_rectified_nobox", rectified_pair[0])
    ##        cv2.imshow("disparity_nobox", disparity)
                            

            moved = False
            for cnt in contours:          
                rect = cv2.minAreaRect(cnt)
                box = cv2.cv.BoxPoints(rect)
                
                # store 4 corners to box    
                box = np.int0(box)
                boxOverlayRectified = box.copy()
                map(lambda x:x[0]+20, boxOverlayRectified)

                # get box area with shoelace formula
                area = 0.0
                for i in range(4):
                    j = (i+1) % 4
                    area += box[i][0] * box[j][1]
                    area -= box[j][0] * box[i][1]
                area = abs(area) / 2.0


                middle = False
                
                for point in box:
                    if (point[1] > (height/3.0) and point[1] < (height*(2/3.0)) and area > minBoxArea):
                        if maxRatio(box, maxLengthWidthRatio) == True:

                            cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                            cv2.drawContours(rectified_pair[0], [boxOverlayRectified], 0, (255, 0, 0), 2)

                            if auto and not moved:
                                avoidBack()
    ##                            cv2.imwrite(str(counter)+"_left_rectified_box"+".jpg", rectified_pair[0])
    ##                            disparity2 = disparity * 255.0
    ##                            disparity2 = disparity2.astype(int)
    ##                            cv2.imwrite(str(counter)+"_disparity_box"+".jpg", disparity2)
                                moved = True
                                

                        
                        middle = True
                        break

                # calculate min and max Y value from tuples
                if not middle:
                    yMin = max(box, key=lambda x:x[1])
                    yMin = yMin[1]
                    yMax = max(box, key=lambda x:x[1])
                    yMax = yMax[1]

                    if yMax > (height*(2/3.0)) and yMin < (height/3.0) and area > minBoxArea:
                        if maxRatio(box, maxLengthWidthRatio) == True:

                            cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                            cv2.drawContours(rectified_pair[0], [boxOverlayRectified], 0, (255, 0, 0), 2)
                            if auto and not moved:
                                avoidBack()
    ##                            cv2.imwrite(str(counter)+"_left_rectified_box"+".jpg", rectified_pair[0])
    ##                            disparity2 = disparity * 255.0
    ##                            disparity2 = disparity2.astype(int)
    ##                            cv2.imwrite(str(counter)+"_disparity_box"+".jpg", disparity2)
                                moved = True


        
##        cv2.imshow("left_unrectified", imgL)

        cv2.imshow("left_rectified_box", rectified_pair[0])
##        cv2.imshow("right_rectified", rectified_pair[1])
        cv2.imshow("disparity_box", disparity)


        

        
##        cv2.imshow("shapeMask", shapeMask)
##        cv2.imshow("disparityProcessed", disparityProcessed)


        # end frame calculation
        endTime = time()
        print("Frame " + str(counter) + " complete -- Time: " + str(endTime - startTime))
        key = cv2.waitKey(1)

        timeData.write(str(endTime - startTime) + "\n")
        timeData.flush()
        # save data to the file
##        if moved:
##            break
##        if counter == 75:
##            break
        
except KeyboardInterrupt:
##        drone.land()
        vcL.release()
        vcR.release()
        pass
vcL.release()
vcR.release()
cv2.destroyAllWindows()
