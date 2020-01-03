import threading
import cv2
from time import sleep
import khani

'''
# ToDo:
    - save file or not
'''

class CameraHandlerSampleImg(threading.Thread):
    def __init__(self, width=640, height=480, saveFramesToFile=True):
        super().__init__()
        # Khani.__init__(self)
        # print(super())
        # print(super().__dir__())

        # Set camera frame size
        self.width = width
        self.height = height

        # Initialization
        # Region Of Interest and Whole scene
        self.sceneWhole = None
        self.sceneROI = None

    def cropROI(self, frame, ROILeft=0, ROITop=50, ROIRight=100, ROIBottom=100):
        # ROI value range: 0 ~ 100
        left = self.width * ROILeft // 100
        top = self.height * ROITop // 100
        right = self.width * ROIRight // 100
        bottom = self.height * ROIBottom // 100
        self.sceneROI = frame[top:bottom, left:right, :]

    def checkROI(self):
        # print("CameraHandler() - checkROI()", type(self.ROI))
        print("checkROI() - CameraHandlerSampleImg(): ", type(self.sceneROI))
        if self.sceneROI.any():
            print('ROI exists')
            print(self.sceneROI.shape)
        else:
            print("ROI not found")
            sleep(5)
            print("Abort")

    def loadSampleImage(self):
        self.sceneWhole = cv2.imread('./img/sample/sample.jpg')

    def saveROIFileFromSampleImage(self):
        self.sceneROI = cv2.imwrite('./img/sample/sampleROI.jpg', self.sceneROI)

    def run(self):
        while True:
            try:
                # Load sample frame from img file
                self.loadSampleImage()

                # Crop ROI from whole scene
                self.cropROI(self.sceneWhole)

                # Check if ROI created
                # self.checkROI()
                # sleep(1)

                # Save to file
                self.saveROIFileFromSampleImage()
            except TypeError:
                print("TypeError - CameraHandlerSampleImg()")
