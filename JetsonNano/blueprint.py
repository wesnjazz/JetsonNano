import threading
import sys
import argparse
import time
from time import sleep
# from jetbot import Robot
from observerManager import Observer, Event
from serialCommunicator import SerialCommunicator
# from cameraHandler import CameraHandler
from cameraHandlerSampleImg import CameraHandlerSampleImg
from laneDetector import LaneDetector
# from robotDriver import RobotDriver

class Khani:
    def __init__(self):
        self.robot = None
        # self.robot = Robot()
        self.serialCommunicator = SerialCommunicator(self)
        self.serialCommunicator.observe('tick tok every 100', self.serialCommunicator.event_triggered)
        # self.cameraHandler = CameraHandler()
        # self.cameraHandlerSampleImg = CameraHandlerSampleImg()
        # self.laneDetector = LaneDetector(self)
        # self.robotDriver = RobotDriver(self)
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        print("Khani():", self)
        self.serialCommunicator.resetTicks()

    # def ObserveEvents(self):
    #

    def startThreads(self):
        self.threadPool.append(self.serialCommunicator)
        # self.threadPool.append(self.cameraHandlerSampleImg)
        # self.threadPool.append(CameraHandler())

        # self.threadPool.append(LaneDetector(self))
        for i in range(len(self.threadPool)):
            self.threadPool[i].start()
        # print("Khani():", self.cameraHandler.ROI)

    def start(self):
        self.startThreads()
        while True:
            # self.laneDetector.detectLanes()
            pass

def main():
    khani = Khani()
    print("Khani() object created. wait for 2 seconds...")
    sleep(2)

    # start threads and kill all threads when Keyboard Interrupt signal (ctrl+c) occurred.
    try:
        khani.start()
    except (KeyboardInterrupt, SystemExit):
        for i in range(len(self.threadPool)):
            self.threadPool[i].join()
            sys.exit()

    # khani.robotDriver.driveXcm(100,0.0,0.0)
    # khani.robotDriver.driveXcm(30,0.3,0.33)
    # khani.robotDriver.turnXLaps()
    # while True:
    #     print("{},{}".format(khani.tickLeft, khani.tickRight))



if __name__ == '__main__':
    main()
