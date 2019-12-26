import threading
from jetbot import Camera
import cv2
from time import sleep

'''
# ToDo:
    - ROI
    - save file or not
'''

class ImageHandler:
    def __init__(self, width = 640, height = 480, saveFramesToFile = True):
        self.width = width
        self.height = height
        self.camera = Camera.instance(width = self.width, height = self.height)
        # sleep(2)
        self.captured_frame = None
        self.fileNumberCount = 1000
        self.saveFramesToFile = saveFramesToFile

    def captureToFile(self, prefix='jpg'):
        # self.save_frames_to_file = True
        self.fileNumberCount += 1
        self.captured_frame = cv2.imwrite('./img/img' + str(self.fileNumberCount) + '.jpg', \
                                                      self.camera.value)
        # print("{}{} saved".format(self.fileNumberCount,'.jpg'))

    def startThread(self):
        self.fileNumberCount = 1000
        while True:
            try:
                self.captureToFile()
            except TypeError:
                print("TypeError")

    def ROI(self):
