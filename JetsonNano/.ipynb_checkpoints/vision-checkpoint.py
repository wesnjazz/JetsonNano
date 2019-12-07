from jetbot import Robot
from jetbot import Camera
import threading
from time import sleep
import cv2

print("wait for 2 seconds")
sleep(2)

camera = Camera.instance(width=640, height=480)

print("num of thread:{}".format(threading.active_count()))

for i in range(20):
    prefix = "img"
    suffix = ".jpg"
    cv2.imwrite(prefix+str(i)+suffix, camera.value)
    sleep(0.5)
    print(prefix+str(i)+suffix+" saved")
    
