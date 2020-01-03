import threading
from jetbot import Camera
import cv2
from time import sleep

'''
# ToDo:
    - save file or not
'''


class CameraHandler(threading.Thread):
    def __init__(self, width=640, height=480, saveFramesToFile=True):
        super().__init__()
        # Set camera frame size
        self.width = width
        self.height = height

        # Initialization
        # Camera instance
        self.camera = Camera.instance(width=self.width, height=self.height)
        # Region Of Interest
        self.sceneWhole = None
        self.sceneROI = None
        # Captured frame
        self.savedFrameWhole = None
        self.savedFrameROI = None
        # File numbering counter
        self.fileNumberCount = 1000000000
        # Flag whether to save to file or not
        self.saveFramesToFile = saveFramesToFile

    def captureToFile(self, area='ROI', fileType='jpg'):
        # self.save_frames_to_file = True
        self.fileNumberCount += 1
        # choose area to be saved to files

        self.savedFrameWhole = cv2.imwrite('./img/cam/cam' + str(self.fileNumberCount) + '.' + fileType, self.sceneWhole)
        self.savedFrameROI = cv2.imwrite('./img/ROI/ROI' + str(self.fileNumberCount) + '.' + fileType, self.sceneROI)

    def cropROI(self, frame, ROILeft=0, ROITop=50, ROIRight=100, ROIBottom=100):
        # ROI value range: 0 ~ 100
        left = self.width * ROILeft // 100
        top = self.height * ROITop // 100
        right = self.width * ROIRight // 100
        bottom = self.height * ROIBottom // 100
        self.sceneROI = frame[top:bottom, left:right, :]
        # self.ROI = self.camera.value[top:bottom, left:right, :]
        # print("CameraHandler()", type(self.ROI), self.ROI.shape)

    def checkROI(self):
        # print("CameraHandler() - checkROI()", type(self.ROI))
        print("checkROI() - CameraHandler(): ", type(self.sceneROI))
        if self.sceneROI.any():
            print(self.sceneROI.shape)
        else:
            print("ROI not found")
            sleep(5)
            print("Abort")

    def run(self):
        self.fileNumberCount = 1000
        while True:
            # sleep(0.1)
            try:
                # Capture the scene from the camera
                #   - This is done automatically in the Camera instance

                # Crop ROI from whole scene
                self.cropROI(self.camera.value)

                # Check whether if there is ROI or not
                # self.checkROI()

                # Save to file
                self.captureToFile(area='ROI')
            except TypeError:
                print("TypeError - CameraHandler()")
