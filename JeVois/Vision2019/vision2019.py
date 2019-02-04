import libjevois as jevois
import cv2
import numpy as np
import math
import time

## Vision 2019 without video streaming
#
#
#
# @author GRT
# 
# @videomapping YUYV 640 480 10 YUYV 640 480 10 GRT Vision2019
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

        self.cameraMatrix = np.array([
                                         [ 502.299385, 0,          320 ],
                                         [ 0,          492.072922, 240 ],
                                         [ 0,          0,          1   ]
                                     ])

        self.objPoints = np.array([
                                      [ -7.313, -4.824, 0 ],
                                      [ -5.377, -5.325, 0 ],
                                      [ -5.936, 0.501,  0 ],
                                      [ -4,     0,      0 ],
                                      [ 5.377,  -5.325, 0 ],
                                      [ 4,      0,      0 ],
                                      [ 5.936,  0.501,  0 ],
                                      [ 7.313, -4.824,  0 ]
                                  ])

        self.distCoeffs = np.array([])

        self.loaded = False

    def loadCalibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if (fs.isOpened()):
            self.cameraMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))

            self.loaded = True
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))
        
    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe=None):
        # Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR. If you need a
        # grayscale image, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB() and getCvRGBA():
        inimg = inframe.getCvBGR()
        h, w, chans = inimg.shape

        if not self.loaded:
            self.loadCalibration(w, h)


        imghsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)
        
        # Start measuring image processing time (NOTE: does not account for input conversion time):
        self.timer.start()

        img = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)

        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))


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
        # cv2.putText(inimg, fps, (3, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))

        # cv2.drawContours(inimg, biggest5, -1, (255,0,0), 3)
        cv2.drawContours(inimg, contours, -1, (255,0,0), 1)
        
        # 2 largest objects
        two_contours = sorted(contours, key = cv2.contourArea, reverse = True)[:2]

        two_present = (len(two_contours) == 2)
        
        cv2.putText(inimg, "GRT 2019 Vision", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),
                     1, cv2.LINE_AA)
        
        valid = True

        points = []

        for contour in two_contours:
            if cv2.contourArea(contour) < 500:
                # INVALID: contour is too small
                valid = False

            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            approx = np.squeeze(approx)
            sums = approx.sum(axis = 1) #sum x and y coords
            bottom_left = np.argmin(sums)
            
            

            # rows,cols = img.shape[:2]
            # vx,vy,x,y = cv2.fitLine(approx, cv2.DIST_L2,0,0.01,0.01)
            # lefty = int((-x*vy/vx) + y)
            # righty = int(((cols-x)*vy/vx)+y)

            # try:
            #     pass
            # except Exception as e:
            #     jevois.sendSerial(str(e))

            rect = cv2.minAreaRect(approx)
            box = cv2.boxPoints(rect)

            box = np.int0(box)

            angle = self.calc_angle(box)
            if not (6 <= abs(math.degrees(angle)) <= 22):
                # INVALID: angle of rectangle not close to 14.5
                jevois.sendSerial("angle")
                valid = False

            # self.draw_text(inimg, math.degrees(angle), box[0])

            #jevois.sendSerial(str(quad_points))
            if len(approx) != 4: #INVALID: polygon doesn't have 4 corners
                
                #jevois.sendSerial(str(box))
                valid = False
            else:
                quad_points = np.array([
                    approx[bottom_left],
                    approx[(bottom_left + 1) % 4],
                    approx[(bottom_left + 2) % 4],
                    approx[(bottom_left + 3) % 4]
                ])
                
                cv2.drawContours(inimg,[quad_points],0,(0,0,255),1)
                
                for point in quad_points:
                    point[1] = h-point[1]

                points.append(quad_points) #USE POLYGON
            
            # jevois.sendSerial(str(box))
            margin = 10
            for v in box:
                if v[0] < margin or v[0] >= w-margin or v[1] < margin or v[1] >= h-margin:
                    # INVALID: too close to image borders
                    jevois.sendSerial("too close")
                    valid = False
                    break

            # points.append(box) #USE RECTANGLE
            # cv2.drawContours(inimg,[box],0,(0,0,255),2)

        # self.draw_text(inimg, contour_area, (30, 115))

            # imgPoints = np.append(imgPoints, box)

        points = np.array(points)

        if not two_present or len(points) < 2:
            # INVALID: only one contour found
            jevois.sendSerial("one present")
            valid = False
        elif abs(points[0][:][1].argmin() - points[1][:][1].argmin()) > 80:
            # INVALID: height difference between two rectangles is too high (should be horizontal)
            jevois.sendSerial("height diff")
            valid = False
        

        # imgPoints = imgPoints.reshape(-1, 2)
        # jevois.sendSerial(str(imgPoints))

        self.draw_text(inimg, valid, (30, 45))

        if valid:
            #comparing x value of first point in both rectangles since order of solvepnp matters
            if points[0][0][0] < points[1][0][0]:
                imgPoints = np.append(np.append(np.array([]), points[0]), points[1])
            else:
                imgPoints = np.append(np.append(np.array([]), points[1]), points[0])

            #imgPoints = imgPoints.reshape(-1, 2)
            imgPoints = np.array(points, dtype=np.float).reshape(8, 2)

            jevois.sendSerial(str(imgPoints))
            jevois.sendSerial(str(self.objPoints))

            retval, revec, tvec = cv2.solvePnP(self.objPoints, imgPoints, self.cameraMatrix, self.distCoeffs)
            
            jevois.sendSerial("{} {} {} {} {} {} {} {}".format(time.time(), retval, tvec[0][0], tvec[1][0], tvec[2][0], revec[0][0], revec[1][0], revec[2][0]))
            # jevois.sendSerial(str(retval))
            # jevois.sendSerial(str(revec))
            # jevois.sendSerial(str(revec))
            # jevois.sendSerial(".   .")
            self.draw_text(inimg, "translation: x{:=5.2f} y{:=3.2f} z{:=3.2f}".format(tvec[0][0], tvec[1][0], tvec[2][0]), (30, 30))
            self.draw_text(inimg, "rotation: x{:=3.2f} y{:=3.2f} z{:=3.2f}".format(math.degrees(revec[0][0]), math.degrees(revec[1][0]), math.degrees(revec[2][0])), (30, 45))


            # cv2.drawContours(inimg, [contour], -1, (255,0,0), 3)

        #jevois.sendSerial(".  .")
        # for c in contours:
            



        # Convert our output image to video output format and send to host over USB:
        if outframe:
            outframe.sendCv(inimg)

    def processNoUSB(self, inframe):
        return process(inframe)
        

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


    def draw_text(self, img, text, point, offset=0):
        cv2.putText(img, str(text), (int(point[0]-20), int(point[1]+20 + offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
        pass
