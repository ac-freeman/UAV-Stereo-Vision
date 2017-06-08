import numpy as np
import cv2
from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.calibration import StereoCalibration
from picamera.array import PiRGBArray
from picamera import PiCamera
from time import sleep, time
from piCams import *
    
camera = PiCamera()
width = 320
height = 240
minBoxArea = 150
res = (width, height)
camera.resolution = res
camera.framerate = 90
##camera.exposure_mode = 'off'
rawCapture = PiRGBArray(camera, size=res)
calibration = StereoCalibration(input_folder="/home/pi/Desktop/programs2017/PiCAM PHOTOS/calibfiles")
block_matcher = StereoBM() # from stereovision.blockmatchers

block_matcher.search_range = 80
block_matcher.bm_preset = 0
block_matcher.window_size = 27
kernel = np.ones((5,5),np.uint8)

setupBoard()
sleep(0.5) # enable camera to start up

counter = 0
try: 
    while True:
        counter += 1
        startT = time()

        cameras("C")
        if counter == 7:
            camera.exposure_mode = 'off'
            camera.shutter_speed = (camera.exposure_speed)
            camera.awb_mode = 'fluorescent'
        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frameC = rawCapture.array


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
        

        rawCapture.truncate(0)
    
        rectified_pair = calibration.rectify((frameA,frameC))

        disparity = block_matcher.get_disparity(rectified_pair)

        disparity = disparity / disparity.max()

        _, shapeMask = cv2.threshold(disparity, 0.8, 1.0, cv2.THRESH_BINARY)
        

        
##        shapeMask = cv2.morphologyEx(shapeMask, cv2.MORPH_CLOSE,kernel)
        disparity3 = shapeMask.copy()
        disparity3 = disparity3 * 255.0
        disparity3 = disparity3.astype(int)

        

        
        disparity3 = cv2.convertScaleAbs(disparity3)
        contours, _ = cv2.findContours(disparity3, 1, 2)

        disparity3 = cv2.cvtColor(disparity3, cv2.COLOR_GRAY2RGB)


        for cnt in contours:          
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)
            
            # store 4 corners to box    
            box = np.int0(box)
            box2 = box.copy()
            map(lambda x:x[0]+20, box2)

            # get box area with hoelace formula
            n = 4
            area = 0.0
            for i in range(n):
                j = (i+1) % n
                area += box[i][0] * box[j][1]
                area -= box[j][0] * box[i][1]
            area = abs(area) / 2.0         

            middle = False
            for point in box:
                if (point[1] > (height/3) and point[1] < (height*(3/2)) and area > minBoxArea):
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                    cv2.drawContours(rectified_pair[0], [box2], 0, (255, 0, 0), 2)
                    middle = True
                    break

            if not middle:
                yMin = max(box, key=lambda x:x[1])
                yMin = yMin[1]
                yMax = max(box, key=lambda x:x[1])
                yMax = yMax[1]


                if yMax > (height*(3/2)) and yMin < (height/3) and area >minBoxArea:
                    cv2.drawContours(disparity, [box], 0, (255, 0, 0), 2)
                    cv2.drawContours(rectified_pair[0],[box2], 0, (255, 0, 0), 2)


        cv2.imshow("a",rectified_pair[0])

        cv2.line(disparity, (0,80), (320,80), .5, 8, 0)
        cv2.line(disparity, (0,160), (320,160), .5, 8, 0)
        endT = time()
        print("Frame " + str(counter) +" complete -- Time: " + str(endT-startT))
        key = cv2.waitKey(1)


        
except KeyboardInterrupt:
    print(disparity)
    print(disparity.max())
    print(shapeMask)
    print(shapeMask.max())
    pass
    
cv2.destroyAllWindows()


