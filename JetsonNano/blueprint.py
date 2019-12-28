import threading
import sys
import argparse
import time
from time import sleep
from jetbot import Robot
from serialCommunicator import SerialCommunicator
from cameraHandler import CameraHandler
from laneDetector import LaneDetector
from robotDriver import RobotDriver

class Khani:
    def __init__(self):
        self.robot = Robot()
        self.serialCommunicator = SerialCommunicator(self)
        self.cameraHandler = CameraHandler()
        self.laneDetector = LaneDetector(self)
        self.robotDriver = RobotDriver(self)
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        self.serialCommunicator.resetTicks()

    def startThreads(self):
        self.threadPool.append(threading.Thread(target=self.serialCommunicator.startThread))
        self.threadPool.append(threading.Thread(target=self.cameraHandler.startThread))
        self.threadPool.append(threading.Thread(target=self.laneDetector.startThread))
        for i in range(len(self.threadPool)):
            self.threadPool[i].start()

def main():
    khani = Khani()
    print("Khani() object created. wait for 2 seconds...")
    sleep(2)

    # start threads and kill all threads when Keyboard Interrupt signal (ctrl+c) occurred.
    try:
        khani.startThreads()
    except (KeyboardInterrupt, SystemExit):
        for i in range(len(self.threadPool)):
            self.threadPool[i].join()
            sys.exit()

    khani.robotDriver.driveXcm(30,0.0,0.0)
    # khani.robotDriver.driveXcm(30,0.3,0.33)
    # khani.robotDriver.turnXLaps()
    while True:
        print("{},{}".format(khani.tickLeft, khani.tickRight))



if __name__ == '__main__':
    main()
