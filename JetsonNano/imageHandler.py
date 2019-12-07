from jetbot import Camera
import cv2
from time import sleep

class ImageHandler:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.camera = Camera.instance(width = self.width, height = self.height)
        sleep(2)
        self.captured_frame = None
        self.count = 1000

    def capture_to_file(self, prefix='jpg'):
        # self.save_frames_to_file = True
        self.count += 1
        self.captured_frame = cv2.imwrite('./img/img' + str(self.count) + '.jpg', \
                                                      self.camera.value)
