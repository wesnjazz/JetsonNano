import threading
import queue as Queue
from collections import deque
import serial
from time import sleep
import time
import cv2
import numpy as np
import copy
import math
import jetbot

class Khani:
    def __init__(self, var):
        self.var = var
        self.serialCommunicator = SerialCommunicator(self, var, printInfo=True)
        self.cameraHandler = CameraHandler(self, var, saveFramesToFile=False)
        # self.cameraHandler = CameraHandlerSampleImg(self, var)
        sleep(2)
        self.laneDetector = LaneDetector(self ,var, printInfo=False, saveFramesToFile=False)
        self.PIDController = PIDController(self, var, printInfo=False)
        # self.serialCommunicator = None
        # self.serialCommunicator.observe('tiktok', self.serialCommunicator.event_triggered)
        # self.cameraHandler = CameraHandler()
        # self.cameraHandlerSampleImg = None
        # self.laneDetector = None
        self.robotDriver = RobotDriver(self, var)

        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        self.serialCommunicator.resetTicks()

    def startThreads(self):
        self.var.tickQueue = Queue.Queue(maxsize = 1)
        self.var.tickLastQueue = Queue.Queue(maxsize = 1)
        self.var.tickDeque = deque(maxlen=2)
        self.var.tickLastDeque = deque(maxlen=1)
        self.var.lock = threading.Lock()
        self.var.eventSerialRead = threading.Event()
        self.var.eventConsumeTicks = threading.Event()
        self.threadPool.append(self.serialCommunicator)
        self.threadPool.append(self.cameraHandler)
        # self.threadPool.append(CameraHandler())
        self.threadPool.append(self.laneDetector)
        self.threadPool.append(self.robotDriver)
        self.threadPool.append(self.PIDController)

        for i in range(len(self.threadPool)):
            self.threadPool[i].start()

        # while True:
        #     sleep(0.001)
        #     print('curTick:{},{}  lastTicl:{},{}'.format(\
        #         self.var.tickLeft, self.var.tickRight, self.var.tickLeftLast, self.var.tickRightLast))

    def start(self):
        self.startThreads()
        # while True:
            # self.laneDetector.detectLanes()
            # pass


class Var:
    def __init__(self, port = '/dev/ttyACM0', baudRate = 9600):
        self.lock = None

        # SerialCommunicator
        self.port = port
        self.baudRate = baudRate
        self.tickQueue = None
        self.tickLastQueue = None
        self.tickDeque = None
        self.tickLastDeque = None
        self.tickEvent = None
        self.eventConsumeTicks = None
        self.tickLeft = 0
        self.tickRight = 0
        self.tickMean = 0
        self.tickLeftLast = 0
        self.tickRightLast = 0
        self.tickMeanLast = 0
        self.readlineDataRaw = None
        self.readlineDataDecoded = None
        self.serialObject = None

        # CameraHandler
        self.camera = None
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
        self.ROIPointYellow = 0
        self.ROIPointWhite = 0
        self.ROIPointRed = 0
        self.centerOfLane = 0
        self.centerOfCamera = 0

        # 1 revolution = 1938 ticks
        # wheel circumference = 20.5cm
        # 1 tick = 20.5cm / 1938 ticks = 0.0106cm = 0.106mm
        # number of ticks 't' to travel 'x' distance:
        # t = x / 0.0106 (in cm)
        self.ticksForOneRevolution = 1938
        self.wheelCircumstance = 20.5
        self.cm_per_tick = self.wheelCircumstance / self.ticksForOneRevolution # 0.0106cm
        self.wheelBase = 11.3

        # Positions
        self.deltaX = 0.0
        self.deltaXLeft = 0.0
        self.deltaXRight = 0.0
        self.deltaY = 0.0
        self.deltaTheta = 0.0
        self.displacementX = 0.0
        self.displacementY = 0.0
        self.displacementTheta = 0.0

        # RobotDriver
        self.robot = None
        self.motorSpeedLeft = 0
        self.motorSpeedRight = 0

        # PID Controller
        self.kP = 1.0
        self.eP = 1.0
        self.kD = 1.0
        self.eD = 1.0
        self.kI = 1.0
        self.eI = 1.0
        self.goalX = 0.0
        self.goalY = 0.0
        self.goalTheta = 0.0
        self.currX = 0.0
        self.currY = 0.0
        self.currTheta = 0.0


class SerialCommunicator (threading.Thread):
    def __init__(self, khani, var, port = '/dev/ttyACM0', baudRate = 9600, printInfo=False):
        super().__init__()
        self.khani = khani
        self.var = var

        # open the serial port
        self.var.serialObject = serial.Serial(self.var.port, self.var.baudRate)
        # flush the serial buffer
        self.var.serialObject.reset_input_buffer()
        self.var.serialObject.reset_output_buffer()

        self.printInfo = printInfo

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

        # self.var.tickLeftLast = self.var.tickLeft
        # self.var.tickRightLast = self.var.tickRightLast

        if len(self.var.readlineDataDecoded) >= 2:
            # print("Serial data has Double DATA")
            for i in range(0, len(self.var.readlineDataDecoded)):
                try:
                    self.var.readlineDataDecoded[i] = int(self.var.readlineDataDecoded[i])
                except ValueError:
                    print("ValueError")
                    self.var.readlineDataDecoded[i] = self.var.tickLeftLast if i == 0 else self.var.tickRightLast
            self.var.tickLeft = self.var.readlineDataDecoded[0]
            self.var.tickRight = self.var.readlineDataDecoded[1]
            return self.var.tickLeft, self.var.tickRight
        else:
            print("Serial data has only SINGLE DATA")
            return self.var.tickLeftLast, self.var.tickRightLast

    def resetTicks(self):
        self.var.serialObject.write(b'r')
        self.var.serialObject.reset_input_buffer()
        self.var.serialObject.reset_output_buffer()

    def run(self):
        self.resetTicks()
        serialcount = 0
        while True:
            sleep(0.01)
            serialcount += 1
            try:
                # self.var.lock.acquire()
                print(serialcount, ' read serial.....', end=' ')
                self.var.tickLeftLast = self.var.tickLeft
                self.var.tickRightLast = self.var.tickRight
                # self.var.tickMeanLast = (self.var.tickLeftLast + self.var.tickRightLast) // 2
                # self.var.tickLastQueue.put((self.var.tickLeftLast, self.var.tickRightLast))
                self.var.tickDeque.append((self.var.tickLeftLast, self.var.tickRightLast))

                self.var.tickLeft, self.var.tickRight = self.readSerial()
                # self.var.tickMean = (self.var.tickLeft + self.var.tickRight) // 2
                self.var.tickDeque.append((self.var.tickLeft, self.var.tickRight))

                self.var.eventSerialRead.set()
                self.var.eventSerialRead.clear()
                self.var.eventConsumeTicks.wait()
                print(' set and clear', end=' ')
                if self.printInfo:
                    print("Current:{},{} - Mean:{}   Last:{},{} - Mean:{}".format(\
                        self.var.tickLeft, self.var.tickRight, self.var.tickMean, \
                        self.var.tickLeftLast, self.var.tickRightLast, self.var.tickMeanLast))

                # self.var.lock.release()
            except TypeError:
                print("TypeError - SerialCommunicator()")


class CameraHandler(threading.Thread):
    def __init__(self, khani, var, width=640, height=480, \
                 ROITop=200, ROIHeight=100, ROILeft=100, ROIWidth=640, saveFramesToFile=True):
        super().__init__()
        self.khani = khani
        self.var = var
        self.var.camWidth = width
        self.var.camHeight = height
        self.var.camera = jetbot.Camera.instance(width=self.var.camWidth, height=self.var.camHeight)
        # Region Of Interest and Whole scene
        self.var.ROITop = ROITop
        self.var.ROIHeight = ROIHeight
        self.var.ROILeft = ROILeft
        self.var.ROIWidth = ROIWidth
        self.saveFramesToFile = saveFramesToFile

    def cropROI(self, scene):
        # cv2.imwrite('./img/scene.jpg', scene)

        # ROI value range: 0 ~ 100
        self.var.ROI = scene[self.var.ROITop:self.var.ROITop + self.var.ROIHeight, \
                       self.var.ROILeft:self.var.ROILeft + self.var.ROIWidth, :]
        # cv2.imwrite('./img/ROI.jpg', self.var.ROI)

    def updateScene(self):
        self.var.scene = self.var.camera.value

    def saveROIFileFromSampleImage(self):
        # save = cv2.imwrite('./img/sample/sampleROI.jpg', self.var.ROI)
        # if save:
        #     self.var.ROI = cv2.imread('./img/sample/sampleROI.jpg')
        cv2.imwrite('./img/ROI.jpg', self.var.ROI)
        cv2.imwrite('./img/sceneROIMarked.jpg', self.var.sceneROIMarked)

    def markROIinScene(self):
        self.var.ROIBorder = 4
        border = self.var.ROIBorder
        top = self.var.ROITop
        height = self.var.ROIHeight
        left = self.var.ROILeft
        width = self.var.ROIWidth

        self.var.sceneROIMarked = copy.deepcopy(self.var.scene)

        # left border
        self.var.sceneROIMarked[top:top+height, left:left+border, 0] = 255
        self.var.sceneROIMarked[top:top+height, left:left+border, 1] = 0
        self.var.sceneROIMarked[top:top+height, left:left+border, 2] = 0
        # right border
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 0] = 255
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 1] = 0
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 2] = 0
        # top border
        self.var.sceneROIMarked[top:top+border, left:left+width, 0] = 255
        self.var.sceneROIMarked[top:top+border, left:left+width, 1] = 0
        self.var.sceneROIMarked[top:top+border, left:left+width, 2] = 0
        # bottom border
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 0] = 255
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 1] = 0
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 2] = 0

    def run(self):
        while True:
            # sleep(1)
            try:
                # Load sample frame from img file
                self.updateScene()

                # Crop ROI from whole scene
                self.cropROI(self.var.scene)
                # Mark ROI in whole scene for debugging
                self.markROIinScene()

                # Save to file
                if self.saveFramesToFile:
                    self.saveROIFileFromSampleImage()
            except TypeError:
                print("TypeError - CameraHandlerSampleImg()")

class CameraHandlerSampleImg(threading.Thread):
    def __init__(self, khani, var, width=640, height=480, \
                 ROITop=200, ROIHeight=240, ROILeft=0, ROIWidth=640, saveFramesToFile=True):
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

    def cropROI(self, scene):
        cv2.imwrite('./img/scene.jpg', scene)

        # ROI value range: 0 ~ 100
        self.var.ROI = scene[self.var.ROITop:self.var.ROITop + self.var.ROIHeight, \
                            self.var.ROILeft:self.var.ROILeft + self.var.ROIWidth, :]
        cv2.imwrite('./img/ROI.jpg', self.var.ROI)

    def loadSampleImage(self):
        self.var.scene = cv2.imread('./img/sample/sample.jpg')

    def saveROIFileFromSampleImage(self):
        save = cv2.imwrite('./img/sample/sampleROI.jpg', self.var.ROI)
        # if save:
        #     self.var.ROI = cv2.imread('./img/sample/sampleROI.jpg')

    def markROIinScene(self):
        self.var.ROIBorder = 2
        border = self.var.ROIBorder
        top = self.var.ROITop
        height = self.var.ROIHeight
        left = self.var.ROILeft
        width = self.var.ROIWidth

        self.var.sceneROIMarked = copy.deepcopy(self.var.scene)

        # left border
        self.var.sceneROIMarked[top:top+height, left:left+border, 0] = 255
        self.var.sceneROIMarked[top:top+height, left:left+border, 1] = 0
        self.var.sceneROIMarked[top:top+height, left:left+border, 2] = 0
        # right border
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 0] = 255
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 1] = 0
        self.var.sceneROIMarked[top:top+height, left+width-border:left+width, 2] = 0
        # top border
        self.var.sceneROIMarked[top:top+border, left:left+width, 0] = 255
        self.var.sceneROIMarked[top:top+border, left:left+width, 1] = 0
        self.var.sceneROIMarked[top:top+border, left:left+width, 2] = 0
        # bottom border
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 0] = 255
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 1] = 0
        self.var.sceneROIMarked[top+height-border:top+height, left:left+width, 2] = 0
        cv2.imwrite('./img/ROI.jpg', self.var.ROI)
        cv2.imwrite('./img/sceneROIMarked.jpg', self.var.sceneROIMarked)

    def run(self):
        while True:
            try:
                # Load sample frame from img file
                self.loadSampleImage()

                # Crop ROI from whole scene
                self.cropROI(self.var.scene)
                # Mark ROI in whole scene for debugging
                self.markROIinScene()

                # Save to file
                # self.saveROIFileFromSampleImage()
            except TypeError:
                print("TypeError - CameraHandlerSampleImg()")


class LaneDetector (threading.Thread):
    def __init__(self, khani, var, printInfo=False, saveFramesToFile=False):
        super().__init__()
        self.khani = khani
        self.var = var

        self.detect_count = 0
        self.printInfo = printInfo
        self.saveFramesToFile = saveFramesToFile

    def detectLanes(self):
        # Convert BGR to HSL
        if self.var.ROI is None:
            if self.printInfo:
                print('ROI is empty', time.time())
            return
        else:
            self.var.ROIHSV = cv2.cvtColor(self.var.ROI, cv2.COLOR_BGR2HSV)

        # Yellow lane
        low_yellow = np.array([20, 100, 100])
        high_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(self.var.ROIHSV, low_yellow, high_yellow)
        yellow = cv2.bitwise_and(self.var.ROI, self.var.ROI, mask=yellow_mask)
        yellow_sum = yellow_mask.sum(axis = 0)
        self.var.ROIPointYellow = yellow_sum.argmax()
        yellow_center = self.var.ROIPointYellow
        yellow[:, yellow_center:yellow_center+self.var.ROIBorder, 0] = 0
        yellow[:, yellow_center:yellow_center+self.var.ROIBorder, 1] = 255
        yellow[:, yellow_center:yellow_center+self.var.ROIBorder, 2] = 255

        # White lane
        low_white = np.array([0, 0, 220])
        high_white = np.array([255, 100, 255])
        white_mask = cv2.inRange(self.var.ROIHSV, low_white, high_white)
        white = cv2.bitwise_and(self.var.ROI, self.var.ROI, mask=white_mask)
        white_sum = white_mask.sum(axis = 0)
        self.var.ROIPointWhite = white_sum.argmax()
        white_center = self.var.ROIPointWhite
        # white[np.arange(len(white)), white_center] = 255
        white[:, white_center:white_center+self.var.ROIBorder, 0] = 255
        white[:, white_center:white_center+self.var.ROIBorder, 1] = 255
        white[:, white_center:white_center+self.var.ROIBorder, 2] = 255

        # Red color
        low_min_red = np.array([0, 100, 100])
        low_max_red = np.array([10, 255, 255])
        high_min_red = np.array([160, 100, 100])
        high_max_red = np.array([179, 255, 255])
        red_low_mask = cv2.inRange(self.var.ROIHSV, low_min_red, low_max_red)
        red_high_mask = cv2.inRange(self.var.ROIHSV, high_min_red, high_max_red)
        red = cv2.bitwise_and(self.var.ROI, self.var.ROI, mask=red_low_mask) + \
              cv2.bitwise_and(self.var.ROI, self.var.ROI, mask=red_high_mask)
        red_sum = red_low_mask.sum(axis = 1) + red_high_mask.sum(axis = 1)
        self.var.ROIPointRed = red_sum.argmax()
        red_center = self.var.ROIPointRed
        if red_center >= 2:
            red[red_center:red_center+self.var.ROIBorder, :, 0] = 255
            red[red_center:red_center+self.var.ROIBorder, :, 1] = 0
            red[red_center:red_center+self.var.ROIBorder, :, 2] = 255

        self.var.centerOfLane = (self.var.ROIPointYellow + self.var.ROIPointWhite) // 2
        self.var.centerOfCamera = (self.var.camWidth // 2) - self.var.ROILeft

        detected_lanes = yellow + white + red
        detected_lanes[:, self.var.centerOfLane:self.var.centerOfLane+self.var.ROIBorder, 0] = 0
        detected_lanes[:, self.var.centerOfLane:self.var.centerOfLane+self.var.ROIBorder, 1] = 255
        detected_lanes[:, self.var.centerOfLane:self.var.centerOfLane+self.var.ROIBorder, 2] = 0
        detected_lanes[:, self.var.centerOfCamera:self.var.centerOfCamera+self.var.ROIBorder, 0] = 0
        detected_lanes[:, self.var.centerOfCamera:self.var.centerOfCamera+self.var.ROIBorder, 1] = 0
        detected_lanes[:, self.var.centerOfCamera:self.var.centerOfCamera+self.var.ROIBorder, 2] = 255

        if self.printInfo:
            print('center of Yellow/White/Red: {:0>3d} / {:0>3d} / {:0>3d}'.format(yellow_center, white_center, red_center))

        if self.saveFramesToFile:
            cv2.imwrite('./img/ROIyellowHSV.jpg', self.var.ROIHSV)
            cv2.imwrite('./img/ROIyellowscene.jpg', self.var.scene)
            cv2.imwrite('./img/ROIyellowresult.jpg', yellow)
            cv2.imwrite('./img/ROIyellowMask.jpg', yellow_mask)
            print('yellow_center:', yellow_center, yellow.shape)

            cv2.imwrite('./img/ROIwhiteHSV.jpg', self.var.ROIHSV)
            cv2.imwrite('./img/ROIwhitescene.jpg', self.var.scene)
            cv2.imwrite('./img/ROIwhiteresult.jpg', white)
            cv2.imwrite('./img/ROIwhiteMask.jpg', white_mask)
            print('white_center:', white_center, white.shape)

            cv2.imwrite('./img/ROIredHSV.jpg', self.var.ROIHSV)
            cv2.imwrite('./img/ROIredscene.jpg', self.var.scene)
            cv2.imwrite('./img/ROIredresult.jpg', red)
            cv2.imwrite('./img/ROIredMasklow.jpg', red_low_mask)
            cv2.imwrite('./img/ROIredMaskhigh.jpg', red_high_mask)

            prefix = './img/detectLanes/lane'
            self.detect_count += 1
            midfix = str(self.detect_count).zfill(6)
            suffix = ".jpg"
            detected_lanes_filename = "{}{}{}".format(prefix, midfix, suffix)
            cv2.imwrite(detected_lanes_filename, detected_lanes)

            prefix = './img/detectLanes/ROI'
            suffix = ".jpg"
            detected_ROI_filename = "{}{}{}".format(prefix, midfix, suffix)
            cv2.imwrite(detected_ROI_filename, self.var.ROI)

    def run(self):
        while True:
            sleep(0.5)
            try:
                self.detectLanes()
                # self.printROI()
                # ROIExists = self.updateROI()
                # while not ROIExists:
                #     sleep(0.2)
                #     print("ROI not exists")
                #     ROIExists = self.updateROI()
                # self.markROIinScene()
                # print("detectLane() ", type(self.ROIMarked), self.ROIMarked.shape)
                pass
            except TypeError:
                print("TypeError - CameraHandler()")


class RobotDriver(threading.Thread):
    def __init__(self, khani, var):
        super().__init__()
        self.khani = khani
        self.var = var
        self.robot = jetbot.Robot()
        self.var.motorSpeedLeft = 0
        self.var.motorSpeedRight = 0

        ###### Ticks ######
        # 1 revolution = 1938 ticks
        # wheel circumference = 20.5cm
        # 1 tick = 20.5cm / 1938 ticks = 0.0106cm = 0.106mm
        # number of ticks 't' to travel 'x' distance:
        # t = x / 0.0106 (in cm)
        self.cm_per_tick = 0.0106

    def driveXcm(self, x, motorSpeedLeft = 0.3, motorSpeedRight = 0.3):
        requiredTicks = self.calculateTicksForXCm(x)
        print("required Ticks:{}".format(requiredTicks))
        # sleep(2)
        while True:
            if self.var.robot is None:
                print('robot is None')
                self.var.robot = jetbot.Robot()
                print('robot created!')
            else:
                print('robot existed')
                break

        print(self.var.robot, self.var.tickLeft, self.var.tickRight)
        goalTicksLeft = self.var.tickLeft + requiredTicks
        goalTicksRight = self.var.tickRight + requiredTicks
        self.var.robot.set_motors(motorSpeedLeft, motorSpeedRight)
        while self.var.tickLeft <= goalTicksLeft and self.var.tickRight <= goalTicksRight:
            pass
        self.var.robot.stop()

    def turnXLaps(self, counterClockwise = True):
        if counterClockwise == True:
            self.var.robot.set_motors(0.3, 0.6)
        sleep(10)
        self.var.robot.stop()

    def calculateTicksForXCm(self, x):
        return x // self.cm_per_tick

    def run(self):
        print('RobotDriver started!!!')
        try:
            self.driveXcm(15,0.3,0.33)
            # self.driveXcm(20,-0.3,-0.33)
            # self.printROI()
            # ROIExists = self.updateROI()
            # while not ROIExists:
            #     sleep(0.2)
            #     print("ROI not exists")
            #     ROIExists = self.updateROI()
            # self.markROIinScene()
            # print("detectLane() ", type(self.ROIMarked), self.ROIMarked.shape)
            pass
        except:
            print("Error - RobotDriver()")


class PIDController(threading.Thread):
    def __init__(self, khani, var, printInfo=False):
        super().__init__()
        self.khani = khani
        self.var = var

        self.printInfo = printInfo
        self.printcount = 0
    def updatePos(self):

        # print(self.var.tickLeft, self.var.tickRight, self.var.cm_per_tick, self.var.wheelBase)
        self.var.deltaXLeft = (self.var.tickLeft - self.var.tickLeftLast) * self.var.cm_per_tick
        self.var.deltaXRight = (self.var.tickRight - self.var.tickRightLast) * self.var.cm_per_tick
        # print(self.var.deltaX, self.var.deltaY, self.var.deltaTheta)
        self.var.deltaX = (self.var.deltaXLeft + self.var.deltaXRight) / 2.0
        self.var.deltaTheta = math.atan2((self.var.deltaXRight - self.var.deltaXLeft) / 2.0, self.var.wheelBase / 2.0)
        self.var.displacementX += self.var.deltaXLeft
        self.var.displacementY += self.var.deltaY
        self.var.displacementTheta += self.var.deltaTheta
        # print(self.var.displacementX, self.var.displacementX, self.var.displacementTheta)

    def printPos(self):
        # print('Tick - current:{},{}  last:{},{}'.format(self.var.tickLeft, self.var.tickRight, \
        #                                                 self.var.tickLeftLast, self.var.tickRightLast))
        print('delta(x/y/Th):{:.2f},{:.2f},{:.2f}  displace:{:.2f},{:.2f},{:.2f}  tick:{},{} ticklast:{},{}'.format(\
            self.var.deltaX, self.var.deltaY, self.var.deltaTheta, \
            self.var.displacementX, self.var.displacementY, self.var.displacementTheta, \
            self.var.tickLeft, self.var.tickRight, self.var.tickLeftLast, self.var.tickRightLast))

    def getErrors(self):
        pass

    def run(self):
        while True:
            # self.updatePos()
            # sleep(0.05)
            self.printcount += 1
            try:
                # self.var.lock.acquire()
                print('wait for the event()...', end=' ')
                self.var.eventSerialRead.wait()
                print('..... event() in PID set')
                ticks = self.var.tickDeque.pop()
                ticksLast = self.var.tickDeque.pop()
                print('self.printcount', self.printcount)
                # ticksLast = self.var.tickLastQueue.get()
                # ticks = self.var.tickQueue.get()
                if ticks is not None:
                    # print('tick', self.var.tickDeque.qsize(), ticks)
                    print('tick', ticks)
                if ticksLast is not None:
                    # print('ticklast', self.var.tickLastDeque.qsize(), ticksLast)
                    print('ticklast', ticksLast)
                    # self.updatePos()
                    # if self.printInfo:
                    #     self.printPos()
                # self.var.lock.release()
                self.var.eventConsumeTicks.set()
                self.var.eventConsumeTicks.clear()
            except IndexError:
                pass
                # print("Error - PIDController()")
