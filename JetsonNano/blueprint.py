import threading
from time import sleep
import time
# from jetbot import Robot
import argparse
from serialCommunicator import SerialCommunicator
# from imageHandler import ImageHandler

class Khani:
    def __init__(self):
        self.serialCommunicator = SerialCommunicator()
        # self.imageHandler = ImageHandler()
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        self.serialCommunicator.resetTicks()

    def threadSerial(self):
        self.serialCommunicator.resetTicks()
        while True:
            try:
                self.tickLeft, self.tickRight = self.serialCommunicator.readSerial()
                print("{},{}".format(self.tickLeft, self.tickRight))
            except TypeError:
                print("TypeError")

    def startThreads(self):
        print("Khani() - start thread")
        self.threadPool.append(threading.Thread(target=self.threadSerial()))
        for i in range(len(self.threadPool)):
            self.threadPool[i].start()
        print("Khani() - start thread")

def main():
    khani = Khani()
    print("main() - start thread")
    khani.startThreads()
    # sleep(2)
    print("main() - finish thread")

if __name__ == '__main__':
    main()
