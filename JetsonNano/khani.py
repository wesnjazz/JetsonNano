import threading
import serial
from time import sleep
import cv2
import numpy as np


class Khani:
    def __init__(self, var):
        self.var = var
        self.serialCommunicator = SerialCommunicator(self, var)
        self.cameraHandler = CameraHandlerSampleImg(self, var)
        sleep(2)
        self.laneDetector = LaneDetector(self ,var)
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
        self.threadPool.append(self.cameraHandler)
        # self.threadPool.append(CameraHandler())
        self.threadPool.append(self.laneDetector)

        for i in range(len(self.threadPool)):
            self.threadPool[i].start()

    def start(self):
        self.startThreads()
        # while True:
            # self.laneDetector.detectLanes()
            # pass


class Var:
    def __init__(self, port = '/dev/ttyACM0', baudRate = 9600):
        # SerialCommunicator
        self.port = port
        self.baudRate = baudRate
        self.tickLeft = 0
        self.tickRight = 0
        self.tickLeftLast = 0
        self.tickRightLast = 0
        self.readlineDataRaw = None
        self.readlineDataDecoded = None
        self.serialObject = None

        # CameraHandler
        self.camWidth = 0
        self.camHeight = 0
        self.scene = None
        self.sceneROIMarked = None
        self.ROI = None
        self.ROIHSV = None
        self.ROITop = 0
        self.ROIHeight = 0
        self.ROILeft = 0
        self.ROIWidth = 0
        self.ROIBorder = 0


class SerialCommunicator (threading.Thread):
    def __init__(self, khani, var, port = '/dev/ttyACM0', baudRate = 9600):
        super().__init__()
        self.khani = khani
        self.var = var
        # self.port = port
        # self.baudRate = baudRate
        # self.tickLeftLast = 0
        # self.tickRightLast = 0
        # self.readlineDataRaw = None
        # self.readlineDataDecoded = None

        # open the serial port
        self.var.serialObject = serial.Serial(self.var.port, self.var.baudRate)
        # flush the serial buffer
        self.var.serialObject.reset_input_buffer()
        self.var.serialObject.reset_output_buffer()

    def readSerial(self):
        # read data from Arduino through serial port
        self.var.readlineDataRaw = self.var.serialObject.readline()
        try:
            # decode the serial data
            self.var.readlineDataDecoded = self.var.readlineDataRaw.decode('utf-8').split(",")
        except UnicodeDecodeError:
            print("UnicodeDecodeError")
            self.var.readlineDataDecoded = [self.var.tickLeftLast, self.var.tickRightLast]
        # if serial data has both left/right ticks
        if len(self.var.readlineDataDecoded) >= 2:
            # print("Serial data has Double DATA")
            for i in range(0, len(self.var.readlineDataDecoded)):
                try:
                    self.var.readlineDataDecoded[i] = int(self.var.readlineDataDecoded[i])
                except ValueError:
                    print("ValueError")
                    self.var.readlineDataDecoded[i] = self.var.tickLeftLast if i == 0 else self.var.tickRightLast
            self.var.tickLeftLast = self.var.readlineDataDecoded[0]
            self.var.tickRightLast = self.var.readlineDataDecoded[1]
            return self.var.readlineDataDecoded[0], self.var.readlineDataDecoded[1]
        else:
            print("Serial data has only SINGLE DATA")
            return self.var.tickLeftLast, self.var.tickRightLast

    def resetTicks(self):
        self.var.serialObject.write(b'r')
        self.var.serialObject.reset_input_buffer()
        self.var.serialObject.reset_output_buffer()

    def run(self):
        self.resetTicks()
        while True:
            try:
                self.var.tickLeft, self.var.tickRight = self.readSerial()
                # print("{},{}".format(self.var.tickLeft, self.var.tickRight))
            except TypeError:
                print("TypeError - SerialCommunicator()")


class CameraHandlerSampleImg(threading.Thread):
    def __init__(self, khani, var, width=640, height=480, \
                 ROITop=240, ROIHeight=239, ROILeft=0, ROIWidth=639, saveFramesToFile=True):
        super().__init__()
        self.khani = khani
        self.var = var
        self.var.camWidth = width
        self.var.camHeight = height
        # Region Of Interest and Whole scene
        self.var.ROITop = ROITop
        self.var.ROIHeight = ROIHeight
        self.var.ROILeft = ROILeft
        self.var.ROIWidth = ROIWidth

    def cropROI(self, frame):
        # ROI value range: 0 ~ 100
        # left = self.width * self.ROILeft // 100
        # top = self.height * self.ROITop // 100
        # right = self.width * self.ROIRight // 100
        # bottom = self.height * self.ROIBottom // 100
        # self.sceneROI = frame[top:bottom, left:right, :]
        self.var.ROI = frame[self.var.ROITop:self.var.ROITop + self.var.ROIHeight, \
                            self.var.ROILeft:self.var.ROILeft + self.var.ROIWidth, :]

    def loadSampleImage(self):
        self.var.scene = cv2.imread('./img/sample/sample.jpg')

    def saveROIFileFromSampleImage(self):
        save = cv2.imwrite('./img/sample/sampleROI.jpg', self.var.ROI)
        if save:
            self.var.ROI = cv2.imread('./img/sample/sampleROI.jpg')

    def run(self):
        while True:
            # print(self.var.camWidth, self.var.camHeight)

            try:
                # Load sample frame from img file
                self.loadSampleImage()

                # Crop ROI from whole scene
                self.cropROI(self.var.scene)

                # Save to file
                self.saveROIFileFromSampleImage()
            except TypeError:
                print("TypeError - CameraHandlerSampleImg()")


class LaneDetector (threading.Thread):
    def __init__(self, khani, var):
        super().__init__()
        self.khani = khani
        self.var = var
        self.cameraHandler = self.khani.cameraHandler

    # def updateROI(self):
    #     if self.var.sceneROI is not None:
    #         print('ROI exists', self.var.sceneROI.shape)
    #         return True
    #     else:
    #         print('ROI not exists')
    #         return False

    def markROI(self):
        self.var.ROIBorder = 2
        top = self.var.ROITop
        height = self.var.ROIHeight
        left = self.var.ROILeft
        width = self.var.ROIWidth

        # print(top, top+height, left, left+border)
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

    def run(self):
        while True:
            sleep(0.8)
            try:
                self.printROI()
                # ROIExists = self.updateROI()
                # while not ROIExists:
                #     sleep(0.2)
                #     print("ROI not exists")
                #     ROIExists = self.updateROI()
                # self.markROI()
                # self.detectLanes()
                # print("detectLane() ", type(self.ROIMarked), self.ROIMarked.shape)
            except TypeError:
                print("TypeError - CameraHandler()")
