import threading
from time import sleep
import time
# from jetbot import Robot
import argparse
from serialCommunicator import SerialCommunicator
from imageHandler import ImageHandler

class Khani:
    def __init__(self):
        self.serialCommunicator = SerialCommunicator(self)
        self.imageHandler = ImageHandler()
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        self.serialCommunicator.resetTicks()

    def startThreads(self):
        self.threadPool.append(threading.Thread(target=self.serialCommunicator.startThread))
        self.threadPool.append(threading.Thread(target=self.imageHandler.startThread))
        for i in range(len(self.threadPool)):
            self.threadPool[i].start()

def main():
    khani = Khani()
    print("Khani() object created. wait for 2 seconds...")
    sleep(2)
    khani.startThreads()
    while True:
        print("{},{}".format(khani.tickLeft, khani.tickRight))
    # sleep(2)

if __name__ == '__main__':
    main()
