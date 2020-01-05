import threading
import serial
from time import sleep
import cv2
import numpy as np


class Khani:
    def __init__(self):
        self.serialCommunicator = SerialCommunicator(self)
        self.cameraHandlerSampleImg = CameraHandlerSampleImg()
        sleep(2)
        self.laneDetector = LaneDetector(self.cameraHandlerSampleImg)
        # self.serialCommunicator = None
        # self.serialCommunicator.observe('tiktok', self.serialCommunicator.event_triggered)
        # self.cameraHandler = CameraHandler()
        # self.cameraHandlerSampleImg = None
        # self.laneDetector = None
        # self.robotDriver = RobotDriver(self)
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        self.serialCommunicator.resetTicks()

    def startThreads(self):
        self.threadPool.append(self.serialCommunicator)
        self.threadPool.append(self.cameraHandlerSampleImg)
        # self.threadPool.append(CameraHandler())
        self.threadPool.append(self.laneDetector)

        for i in range(len(self.threadPool)):
            self.threadPool[i].start()

    def start(self):
        self.startThreads()
        # while True:
            # self.laneDetector.detectLanes()
            # pass


class SerialCommunicator (threading.Thread):
    def __init__(self, khani, port = '/dev/ttyACM0', baudRate = 9600):
        super().__init__()
        self.khani = khani
        self.port = port
        self.baudRate = baudRate
        self.lastTickLeft = 0
        self.lastTickRight = 0
        self.readlineDataRaw = None
        self.readlineDataDecoded = None

        # open the serial port
        self.serialObject = serial.Serial(self.port, self.baudRate)
        # flush the serial buffer
        self.serialObject.reset_input_buffer()
        self.serialObject.reset_output_buffer()

    def readSerial(self):
        # read data from Arduino through serial port
        self.readlineDataRaw = self.serialObject.readline()
        try:
            # decode the serial data
            self.readlineDataDecoded = self.readlineDataRaw.decode('utf-8').split(",")
        except UnicodeDecodeError:
            print("UnicodeDecodeError")
            self.readlineDataDecoded = [self.lastTickLeft, self.lastTickRight]
        # if serial data has both left/right ticks
        if len(self.readlineDataDecoded) >= 2:
            # print("Serial data has Double DATA")
            for i in range(0, len(self.readlineDataDecoded)):
                try:
                    self.readlineDataDecoded[i] = int(self.readlineDataDecoded[i])
                except ValueError:
                    print("ValueError")
                    self.readlineDataDecoded[i] = self.lastTickLeft if i == 0 else self.lastTickRight
            self.lastTickLeft = self.readlineDataDecoded[0]
            self.lastTickRight = self.readlineDataDecoded[1]
            return self.readlineDataDecoded[0], self.readlineDataDecoded[1]
        else:
            print("Serial data has only SINGLE DATA")
            return self.lastTickLeft, self.lastTickRight

    def resetTicks(self):
        self.serialObject.write(b'r')
        self.serialObject.reset_input_buffer()
        self.serialObject.reset_output_buffer()

    def run(self):
        self.resetTicks()
        while True:
            try:
                self.khani.tickLeft, self.khani.tickRight = self.readSerial()
                # print("{},{}".format(self.khani.tickLeft, self.khani.tickRight))
            except TypeError:
                print("TypeError - SerialCommunicator()")


class CameraHandlerSampleImg(threading.Thread):
    def __init__(self, width=640, height=480, \
                 ROITop=240, ROIHeight=239, ROILeft=0, ROIWidth=639, saveFramesToFile=True):
        super().__init__()

        # Set camera frame size
        self.width = width
        self.height = height

        # Initialization
        # Region Of Interest and Whole scene
        self.sceneWhole = None
        self.sceneROI = None
        self.ROITop = ROITop
        self.ROIHeight = ROIHeight
        self.ROILeft = ROILeft
        self.ROIWidth = ROIWidth

    def cropROI(self, frame):
        # ROI value range: 0 ~ 100
        # left = self.width * self.ROILeft // 100
        # top = self.height * self.ROITop // 100
        # right = self.width * self.ROIRight // 100
        # bottom = self.height * self.ROIBottom // 100
        # self.sceneROI = frame[top:bottom, left:right, :]
        self.sceneROI = frame[self.ROITop:self.ROITop + self.ROIHeight, self.ROILeft:self.ROILeft + self.ROIWidth, :]

    def loadSampleImage(self):
        self.sceneWhole = cv2.imread('./img/sample/sample.jpg')

    def saveROIFileFromSampleImage(self):
        save = cv2.imwrite('./img/sample/sampleROI.jpg', self.sceneROI)
        if save:
            self.sceneROI = cv2.imread('./img/sample/sampleROI.jpg')

    def run(self):
        while True:
            try:
                # Load sample frame from img file
                self.loadSampleImage()

                # Crop ROI from whole scene
                self.cropROI(self.sceneWhole)

                # Save to file
                self.saveROIFileFromSampleImage()
            except TypeError:
                print("TypeError - CameraHandlerSampleImg()")


class LaneDetector (threading.Thread):
    def __init__(self, cameraHandler):
        super().__init__()
        self.cameraHandler = cameraHandler
        self.ROI = None
        self.ROIMarked = None
        self.ROIHSV = None

    def detectLanes(self):
        # Convert BGR to HSL
        self.ROIHSV = cv2.cvtColor(self.ROI, cv2.COLOR_BGR2HSV)

        # Range for lower red
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(self.ROIHSV, lower_red, upper_red)

        # Range for upper range
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(self.ROIHSV, lower_red, upper_red)

        # # Generating the final mask to detect red color
        # mask = mask1 + mask2
        #
        # mask1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # mask1 = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3, 3), np.uint8))
        #
        # # creating an inverted mask to segment out the cloth from the frame
        # mask2 = cv2.bitwise_not(mask1)
        #
        # # Segmenting the cloth out of the frame using bitwise and with the inverted mask
        # res1 = cv2.bitwise_and(self.ROIMarked, self.ROIMarked, mask=mask2)
        #
        # # creating image showing static background frame pixels only for the masked region
        # res2 = cv2.bitwise_and(self.ROI, self.ROI, mask=mask1)
        #
        # # Generating the final output
        # final_output = cv2.addWeighted(res1, 1, res2, 1, 0)



        lower_yellow = np.array([0, 30, 80])
        upper_yellow = np.array([20, 70, 120])

        mask = cv2.inRange(self.ROIHSV, lower_yellow, upper_yellow)
        result = cv2.bitwise_and(self.ROI, self.ROI, mask=mask)

        # a = np.zeros([3,3,3])
        # print(type(a))
        # print(a.shape)
        # a[:,:3,0] = 255

        # cv2.imwrite('./img/a.jpg', a)
        # cv2.imwrite('./img/ROIfinal.jpg', final_output)
        cv2.imwrite('./img/ROI.jpg', self.ROI)
        cv2.imwrite('./img/ROIMarked.jpg', self.ROIMarked)
        cv2.imwrite('./img/ROIHSV.jpg', self.ROIHSV)
        cv2.imwrite('./img/ROIMask.jpg', mask)
        cv2.imwrite('./img/ROIResult.jpg', result)

    def updateROI(self):
        self.ROI = self.cameraHandler.sceneROI
        # print(type(self.ROI))
        return True if self.ROI is not None else False

    def markROI(self):
        border = 2
        top = self.cameraHandler.ROITop
        height = self.cameraHandler.ROIHeight
        # bottom = self.cameraHandler.ROITop + self.cameraHandler.ROIHeight
        left = self.cameraHandler.ROILeft
        width = self.cameraHandler.ROIWidth
        # right = self.cameraHandler.ROILeft + self.cameraHandler.ROIWidth

        self.ROIMarked = self.ROI
        # print(self.ROIMarked.shape, self.ROI.shape)
        # mark left border

        print(top, top+height, left, left+border)
        self.ROIMarked[0:height, 0:border, 0] = 0
        self.ROIMarked[0:height, 0:border, 1] = 0
        self.ROIMarked[0:height, 0:border, 2] = 255
        # mark right border
        self.ROIMarked[0:height, width-border:, 0] = 0
        self.ROIMarked[0:height, width-border:, 1] = 0
        self.ROIMarked[0:height, width-border:, 2] = 255
        # mark top border
        self.ROIMarked[0:border, :, 0] = 0
        self.ROIMarked[0:border, :, 1] = 0
        self.ROIMarked[0:border, :, 2] = 255
        # mark bottom border
        self.ROIMarked[height - border:, :, 0] = 0
        self.ROIMarked[height - border:, :, 1] = 0
        self.ROIMarked[height - border:, :, 2] = 255
        # self.ROIMarked[50,50,0] = 0
        # self.ROIMarked[50,50,0] = 255
        # self.ROIMarked[50,50,0] = 255
        # self.ROIMarked[:,0:10,0] = 0
        # self.ROIMarked[:,0:10,1] = 0
        # self.ROIMarked[:,0:10,2] = 0
        cv2.imwrite('./img/mark.jpg', self.ROIMarked)
        cv2.imwrite('./img/roi.jpg', self.ROI)

        # print(self.ROIMarked.shape)
        # print(top, height, left, width)
        # print(self.ROIMarked[top, left, 0])

    def run(self):
        while True:
            sleep(0.2)
            try:
                ROIExists = self.updateROI()
                while not ROIExists:
                    sleep(0.2)
                    print("ROI not exists")
                    ROIExists = self.updateROI()
                self.markROI()
                # self.detectLanes()
                # print("detectLane() ", type(self.ROIMarked), self.ROIMarked.shape)
            except TypeError:
                print("TypeError - CameraHandler()")
