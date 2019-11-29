from jetbot import Robot
from jetbot import Camera
from jetbot import bgr8_to_jpeg

from time import sleep
#import ipywidgets.widgets as widgets
#import traitlets

import numpy as np
import cv2


camera = Camera.instance()
r, f = camera.cap.read()

# while(camera.cap.isOpened()):  # check !
#     # capture frame-by-frame
#     ret, frame = camera.cap.read()
# 
#     if ret: # check ! (some webcam's need a "warmup")
#         # our operation on frame come here
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# 
#         # Display the resulting frame
#         cv2.imshow('frame', gray)
#         cv2.waitKey(0)
#         cv2.destroyWindow('frame')
#     else:
#         print('image frame not detected')
# 
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# # When everything is done release the capture
# camera.cap.release()
# cv2.destroyAllWindows()
# 
