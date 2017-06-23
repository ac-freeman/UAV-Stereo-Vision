import numpy as np
import cv2
from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.calibration import StereoCalibration
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep, time
from piCams import *

# set up camera object
# interface with one port
camera = PiCamera()
width = 640
height = 480
minBoxArea = 150
res = (width, height)
camera.resolution = res
camera.framerate = 90

rawCapture = PiRGBArray(camera, size=res)
calibration = StereoCalibration(input_folder = "/home/pi/Desktop/programs2017/UAV-Stereo-Vision/calib7/calibfiles")
block_matcher = StereoBM() # from stereovision.blockmatchers

block_matcher.search_range = 80
block_matcher.bm_preset = 0
block_matcher.window_size = 27
kernel = np.ones((5,5),np.uint8)

setupBoard()
sleep(0.5) # enable camera to start up

counter = 0

# write frame time to a txt file
timeData = open("timeData.txt", 'w')
# clear old data
timeData.truncate()

try: 
    while True:
        counter += 1
        # begin frame calculation
        startTime = time()

        # Right frame
        cameras("C")
        if counter == 7:
            camera.exposure_mode = 'off'
            camera.shutter_speed = (camera.exposure_speed)
            camera.awb_mode = 'fluorescent'
            print("auto exposure off")

        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameC = rawCapture.array
        rawCapture.truncate(0)

        sleep(0.03)

        # Left frame
        cameras("A")
        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameA = rawCapture.array
        rawCapture.truncate(0)
    
        rectified_pair = calibration.rectify((frameA,frameC))

        disparity = block_matcher.get_disparity(rectified_pair)
        disparity = disparity / disparity.max()
        # normalize disparity in [0, 1]

        _, shapeMask = cv2.threshold(disparity, 0.8, 1.0, cv2.THRESH_BINARY)
        
##        shapeMask = cv2.morphologyEx(shapeMask, cv2.MORPH_CLOSE,kernel)
        disparityProcessed = shapeMask.copy()
        disparityProcessed = disparityProcessed * 255.0
        disparityProcessed = disparityProcessed.astype(int)
        # bring into 255 range

        disparityProcessed = cv2.convertScaleAbs(disparityProcessed)
        contours, _ = cv2.findContours(disparityProcessed, 1, 2)
        # find contours from thresholded disparity map

        disparityProcessed = cv2.cvtColor(disparityProcessed, cv2.COLOR_GRAY2RGB)


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
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                    cv2.drawContours(rectified_pair[0], [boxOverlayRectified], 0, (255, 0, 0), 2)
                    middle = True
                    break

            # calculate min and max Y value from tuples
            if not middle:
                yMin = max(box, key=lambda x:x[1])
                yMin = yMin[1]
                yMax = max(box, key=lambda x:x[1])
                yMax = yMax[1]


                if yMax > (height*(2/3.0)) and yMin < (height/3.0) and area > minBoxArea:
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                    cv2.drawContours(rectified_pair[0],[boxOverlayRectified], 0, (255, 0, 0), 2)

##        cv2.imshow("frameA", frameA)
##        cv2.imshow("frameC", frameC)



        # plot line on disparity map
        # points, color, thickness, type of line (8 connected), # of fractional bits in coordinates
        cv2.line(disparity, (0,80), (320,80), 0.5, 8, 0)
        cv2.line(disparity, (0,160), (320,160), 0.5, 8, 0)


        cv2.imshow("a", rectified_pair[0])
        cv2.imshow("c", rectified_pair[1])
        cv2.imshow("disparity", disparity)

        # end frame calculation
        endTime = time()
        print("Frame " + str(counter) + " complete -- Time: " + str(endTime - startTime))
        key = cv2.waitKey(1)

        timeData.write(str(endTime - startTime) + "\n")
        timeData.flush()
        # save data to the file


except KeyboardInterrupt:
    # quit on Ctrl-C
    pass
    
cv2.destroyAllWindows()


