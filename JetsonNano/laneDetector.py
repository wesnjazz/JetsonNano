import threading
import cv2
import numpy as np
from time import sleep
# from khani import Khani

'''
# ToDo:
    - ROI
    - save file or not
'''

class LaneDetector (threading.Thread):
    def __init__(self, khani):
        super().__init__()
        # print("LaneDetector() created")
        self.khani = khani
        print("arg:", khani, "self:", self.khani)
        self.ROIMarked = self.khani.cameraHandlerSampleImg.sceneROI
        self.ROIHSL = None
        print("LaneDetector():", self.khani.cameraHandlerSampleImg.sceneROI)
        print("LaneDetector():", self.ROIMarked)


    def detectLanes(self):
        # Convert BGR to HSL
        # if self.ROIMarked == None:
            # raise Exception('ROIMarked is None')
            # print("detect Lanes() ROIMakred is None")
            # return None
        # print("detect Lanes() yeah!!!")
        # self.ROIHSL = cv2.cvtColor(self.ROIMarked, cv2.COLOR_BGR2HLS)
        sleep(1)
        # self.khani.cameraHandlerSampleImg.checkROI()
        # self.checkROI()
        if self.ROIMarked == None:
            print('self.ROIMarked == None')
            return
        self.ROIHSL = self.ROIMarked
        if self.ROIHSL.any():
            print("ROIHSL converted!")
        else:
            print("ROIHSL not exists")

        lower_yellow = np.array([40, 70, 40])
        upper_yellow = np.array([70, 100, 60])

        mask = cv2.inRange(self.ROIHSL, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(self.ROIMarked, self.ROIMarked, mask=mask)

        cv2.imwrite('./img/ROIMarked.jpg', self.ROIMarked)
        cv2.imwrite('./img/ROIHSL.jpg', self.ROIHSL)
        cv2.imwrite('./img/ROIMask.jpg', mask)
        cv2.imwrite('./img/ROIRes.jpg', res)


        # width = 2
        # top = self.ROItop
        # bottom = self.ROItop + self.ROIheight
        # left = self.ROIleft
        # right = self.ROIleft + self.ROIwidth
        #
        # # mark left border
        # self.markedImage[top:bottom][left:left+width][0] = 255
        # self.markedImage[top:bottom][left:left+width][1] = 0
        # self.markedImage[top:bottom][left:left+width][2] = 0
        # # mark right border
        # self.markedImage[top:bottom][right-width:right][0] = 255
        # self.markedImage[top:bottom][right-width:right][1] = 0
        # self.markedImage[top:bottom][right-width:right][2] = 0
        # # mark top border
        # self.markedImage[top:top+width][left:right][0] = 255
        # self.markedImage[top:top+width][left:right][1] = 0
        # self.markedImage[top:top+width][left:right][2] = 0
        # # mark bottom border
        # self.markedImage[bottom:bottom-width][left:right][0] = 255
        # self.markedImage[bottom:bottom-width][left:right][1] = 0
        # self.markedImage[bottom:bottom-width][left:right][2] = 0

    def checkROI(self):
        print("checkROI() - LaneDetector(): ", type(self.ROIMarked))
        if self.ROIMarked.any() == True:
            print(self.ROIMarked.shape)

    def run(self):
        while True:
            # sleep(0.1)
            try:
                self.checkROI()
                self.khani.cameraHandlerSampleImg.checkROI()
                # self.detectLanes()
                # print("detectLane() ", type(self.ROIMarked), self.ROIMarked.shape)
            except TypeError:
                print("TypeError - CameraHandler()")
