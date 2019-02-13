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
        self.objPoints = [
                                      [ -7.313, 4.824,  0 ], # min x
                                      [ -4,     0,      0 ], # max x
                                      [ -5.936, -0.501, 0 ], # min y
                                      [ -5.377, 5.325,  0 ], # max y
                                      [ 4,      0,      0 ], # min x
                                      [ 7.313, 4.824,   0 ], # max x
                                      [ 5.936,  -0.501, 0 ], # min y
                                      [ 5.377,  5.325,  0 ]  # max y
                        ]
        self.objPoints = np.array(self.objPoints)
        jevois.sendSerial(str(self.objPoints))

        self.distCoeffs = np.array([])

        self.loaded = False

        self.outframe = None

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
        self.outframe = outframe

        # Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR. If you need a
        # grayscale image, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB() and getCvRGBA():
        inimg = inframe.getCvBGR()
        h, w, chans = inimg.shape

        if not self.loaded:
            self.loadCalibration(w, h)


        imghsv = cv2.cvtColor(inimg, cv2.COLOR_BGR2HSV)

        img = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)
        inimg = cv2.bitwise_and(inimg, inimg, mask=cv2.bitwise_not(img))

        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))


        img = cv2.erode(img, self.erodeElement)
        img = cv2.dilate(img, self.dilateElement)

        contours, hierarchy = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        two_contours = sorted(contours, key = cv2.contourArea, reverse = True)[:2]

        two_present = (len(two_contours) == 2)
        
        self.putText(inimg, "GRT 2019 Vision", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),
                     1, cv2.LINE_AA)
        
        valid = True

        points = np.ndarray((8, 2), dtype=np.float32)

        left_contour = None
        right_contour = None
        if two_present:
            if two_contours[0][0, 0, 0] < two_contours[1][0, 0, 0]:
                left_contour = two_contours[0]
                right_contour = two_contours[1]
            else:
                left_contour = two_contours[1]
                right_contour = two_contours[0]
            self.drawContours(inimg, [left_contour, right_contour], -1, (0, 255, 255), thickness=-1)

            left_contour = np.squeeze(left_contour)
            points[0] = self.minmax(left_contour, 0, 1, -1, 1) # min x
            points[1] = self.minmax(left_contour, 0, 1, 1, -1) # max x
            points[2] = self.minmax(left_contour, 1, 0, -1, -1) # min y
            points[3] = self.minmax(left_contour, 1, 0, 1, 1) # max y
            right_contour = np.squeeze(right_contour)
            points[4] = self.minmax(right_contour, 0, 1, -1, -1) # min x
            points[5] = self.minmax(right_contour, 0, 1, 1, 1) # max x
            points[6] = self.minmax(right_contour, 1, 0, -1, 1) # min y
            points[7] = self.minmax(right_contour, 1, 0, 1, -1) # max y

            angles = np.ndarray((4,), dtype=np.float32)
            angles[0] = self.calc_angle(points[2], points[0])
            angles[1] = self.calc_angle(points[1], points[3])
            angles[2] = self.calc_angle(points[4], points[7])
            angles[3] = self.calc_angle(points[6], points[5])
            if np.amax(np.abs(angles - 14.5)) > 5:
                valid = False
        else:
            valid = False

        for contour in two_contours:
            if cv2.contourArea(contour) < 500:
                # INVALID: contour is too small
                valid = False
        
        margin = 10
        for v in points:
            if v[0] < margin or v[0] >= w-margin or v[1] < margin or v[1] >= h-margin:
                valid = False

        points = np.array(points, dtype=np.float32)
        for i in range(len(points)):
            self.drawCircle(inimg, (points[i,0], points[i,1]), 1, (255, 0, 0), 1)

        if len(points) != 8:
            valid = False

        if valid:
            retval, revec, tvec, inliers = cv2.solvePnPRansac(self.objPoints, points, self.cameraMatrix, self.distCoeffs)

            
            #jevois.sendSerial("{} {} {} {} {} {} {} {}".format(time.time(), retval, tvec[0][0], tvec[1][0], tvec[2][0], revec[0][0], revec[1][0], revec[2][0]))
            self.draw_text(inimg, "translation: x{:=5.2f} y{:=3.2f} z{:=3.2f}".format(tvec[0][0], tvec[1][0], tvec[2][0]), (30, 30))
            self.draw_text(inimg, "rotation: x{:=3.2f} y{:=3.2f} z{:=3.2f}".format(math.degrees(revec[0][0]), math.degrees(revec[1][0]), math.degrees(revec[2][0])), (30, 45))


        if outframe:
            outframe.sendCv(inimg)

    def processNoUSB(self, inframe):
        return self.process(inframe)


    # METHODS FOR Vision2019 WITH VIDEO OUTPUT (debug only)

    def drawContours(self, *args, **kwargs):
        if self.outframe:
            cv2.drawContours(*args, **kwargs)

    def drawCircle(self, *args, **kwargs):
        if self.outframe:
            cv2.circle(*args, **kwargs)

    def putText(self, *args, **kwargs):
        if self.outframe:
            cv2.putText(*args, **kwargs)

    def sendSerial(self, *args, **kwargs):
        if self.outframe:
            jevois.sendSerial(*args, **kwargs)

    ###
        

    def calc_angle(self, pt1, pt2):
        """
        Returns the angle a rectangle is pointing towards

        Points should be ordered so that lowest is first, rest follow counter clockwise
        
        Down is 0, right is pi/2
        -pi/2 <= n <= pi/2
        """
        return abs(math.atan2(pt2[0] - pt1[0], pt2[1] - pt1[1]) * 180 / math.pi)

    def calc_dist(self, point1, point2):
        return math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2))


    def draw_text(self, img, text, point, offset=0):
        cv2.putText(img, str(text), (int(point[0]-20), int(point[1]+20 + offset)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
        pass

    def minmax(self, arr, major_axis, minor_axis, dir1, dir2, big_number = 1000.0):
        scores = arr[:, major_axis] * dir1 + arr[:, minor_axis] * (dir2 / big_number)
        return arr[scores.argmax()]
