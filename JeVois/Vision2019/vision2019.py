import libjevois as jevois
import cv2
import numpy as np
import math

## Detect hatch vision tape in Deep Space 2019
#
#
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
        self.HSVmin = np.array([ 0,  50, 160], dtype=np.uint8)
        self.HSVmax = np.array([ 255, 255, 255], dtype=np.uint8)

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
        
        # 2 largest objects
        two_contours = sorted(contours, key = cv2.contourArea, reverse = True)[:2]

        two_present = (len(two_contours) == 2)
        
        cv2.putText(inimg, "JeVois Python Sandbox", (3, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),
                     1, cv2.LINE_AA)
        
        for contour in two_contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, False)
            rows,cols = img.shape[:2]
            vx,vy,x,y = cv2.fitLine(approx, cv2.DIST_L2,0,0.01,0.01)
            lefty = int((-x*vy/vx) + y)
            righty = int(((cols-x)*vy/vx)+y)

            try:
                rect = cv2.minAreaRect(approx)
                box = cv2.boxPoints(rect)

                box = np.int0(box)
                cv2.drawContours(inimg,[box],0,(0,0,255),2)

                angle = self.calc_angle(box)
                self.draw_text(inimg, math.degrees(angle), box[0])

                jevois.sendSerial(str(box))
            except Exception as e:
                jevois.sendSerial(str(e))

            #cv2.drawContours(inimg, [contour], -1, (255,0,0), 3)

        #jevois.sendSerial(".  .")
        # for c in contours:
            



        # Convert our output image to video output format and send to host over USB:
        outframe.sendCv(inimg)
        

    def calc_angle(self, points):
        """
        Returns the angle a rectangle is pointing towards

        Points should be ordered so that lowest is first, rest follow counter clockwise
        
        Down is 0, right is pi/2
        -pi/2 <= n <= pi/2
        """
        lowest_point = points[0]
        far_point = None
        if (self.calc_dist(points[0], points[1]) > self.calc_dist(points[0], points[3])):
            far_point = points[1]
        else:
            far_point = points[3]

        return math.atan( (lowest_point[0] - far_point[0]) / abs(lowest_point[1] - far_point[1]) )


    def calc_dist(self, point1, point2):
        return math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2))


    def draw_text(self, img, text, point):
        cv2.putText(img, str(text), (point[0]-20, point[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
