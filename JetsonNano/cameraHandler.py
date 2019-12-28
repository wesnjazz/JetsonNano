import threading
from jetbot import Camera
import cv2
from time import sleep

'''
# ToDo:
    - save file or not
'''

class CameraHandler:
    def __init__(self, width = 640, height = 480, saveFramesToFile = True):
        self.width = width
        self.height = height
        self.camera = Camera.instance(width = self.width, height = self.height)
        # sleep(2)
        self.captured_frame = None
        self.fileNumberCount = 1000000000
        self.saveFramesToFile = saveFramesToFile
        self.ROI = None

    def captureToFile(self, area='ROI', fileType='jpg'):
        # self.save_frames_to_file = True
        self.fileNumberCount += 1
        # choose area to be saved to files
        scene = self.ROI if area == 'ROI' else self.camera.value
        self.captured_frame = cv2.imwrite('./img/img' + str(self.fileNumberCount) + '.' + fileType, scene)
        # print("{}{} saved".format(self.fileNumberCount,'.jpg'))

    def cropROI(self, ROILeft=0, ROITop=50, ROIRight=100, ROIBottom=100):
        # ROI value range: 0 ~ 100
        left = self.width * ROILeft // 100
        top = self.height * ROITop // 100
        right = self.width * ROIRight // 100
        bottom = self.height * ROIBottom // 100
        self.ROI = self.camera.value[top:bottom, left:right, :]

    def startThread(self):
        self.fileNumberCount = 1000
        while True:
            try:
                self.cropROI()
                self.captureToFile(area='ROI')
            except TypeError:
                print("TypeError - CameraHandler()")
