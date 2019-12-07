from jetbot import Robot
from jetbot import Camera
import threading
from time import sleep
import cv2


camera = Camera.instance(width=640, height=480)

print("num of thread:{}".format(threading.active_count()))
print("wait for 2 seconds")
sleep(2)

robot = Robot()
robot.set_motors(0.3, 0.38)
for i in range(40):
    prefix = "./img/img"
    suffix = ".jpg"
    cv2.imwrite(prefix+str(i)+suffix, camera.value)
    sleep(0.3)
    print(prefix+str(i)+suffix+" saved")
robot.stop()
