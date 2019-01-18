import libjevois as jevois
import cv2
import numpy as np
import math

## Detect tape from "alignment lines"
#
# Add some description of your module here.
#
# @author GRT
# 
# @videomapping YUYV 640 480 30 YUYV 640 480 30 GRT Vision2019
# @email gunnrobotics192@gmail.com
# @address 123 first street, Los Angeles CA 90012, USA
# @copyright Copyright (C) 2018 by GRT
# @mainurl gunnrobotics.com
# @supporturl gunnrobotics.com
# @otherurl gunnrobotics.com
# @license 
# @distribution Unrestricted
# @restrictions None
# @ingroup modules
class Vision2019:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        print(cv2.__version__)
        self.HSVmin = np.array([ 0,  0, 180], dtype=np.uint8)
        self.HSVmax = np.array([ 255, 50, 255], dtype=np.uint8)

        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("processing timer", 100, jevois.LOG_INFO)
        
    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe=None):
        # Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR. If you need a
        # grayscale image, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB() and getCvRGBA():
        inimg = inframe.getCvBGR()


        imghsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()

        img = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)

        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))


        img = cv2.erode(img, self.erodeElement)
        img = cv2.dilate(img, self.dilateElement)

        contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # biggest5 = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        # Write a title:
        # cv2.putText(outimg, "JeVois VisionTestChris", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()
        # height = outimg.shape[0]
        # width = outimg.shape[1]
        # cv2.putText(outimg, fps, (3, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))

        # cv2.drawContours(inimg, biggest5, -1, (255,0,0), 3)
        # cv2.drawContours(inimg, contours, -1, (255,0,0), 3)
        
        # 5 largest objects
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:5]

        if contours:
            largest = contours[0]
            # epsilon = 0.03 * cv2.arcLength(largest, True)
            epsilon = 0.005 * cv2.arcLength(largest, True)
            approx = cv2.approxPolyDP(largest, epsilon, False)
            rows,cols = img.shape[:2]
            vx,vy,x,y = cv2.fitLine(approx, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)

            try:
                cv2.line(inimg,(cols-1,righty),(0,lefty),(0,255,0),2)
            except:
                pass

            cv2.drawContours(inimg, [approx], -1, (255,0,0), 3)

        # for c in contours:
            



        # Convert our output image to video output format and send to host over USB:
        outframe.sendCv(inimg)
        
