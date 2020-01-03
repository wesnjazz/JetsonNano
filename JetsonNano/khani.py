# from jetbot import Robot
# import observerManager
# from cameraHandler import CameraHandler
# import cameraHandlerSampleImg
# import laneDetector
# from robotDriver import RobotDriver


class Khani:
    def __init__(self):
        self.robot = None
        # self.robot = Robot()
        import serialCommunicator
        self.serialCommunicator = serialCommunicator.SerialCommunicator()
        # self.serialCommunicator.observe('tiktok', self.serialCommunicator.event_triggered)
        # self.cameraHandler = CameraHandler()
        # self.cameraHandlerSampleImg = cameraHandlerSampleImg.CameraHandlerSampleImg()
        # self.laneDetector = LaneDetector(self)
        # self.robotDriver = RobotDriver(self)
        self.threadPool = []
        self.tickLeft = 0
        self.tickRight = 0

        print("Khani():", self)
        self.serialCommunicator.resetTicks()

    def startThreads(self):
        self.threadPool.append(self.serialCommunicator)
        # self.threadPool.append(self.cameraHandlerSampleImg)
        # self.threadPool.append(CameraHandler())

        # self.threadPool.append(self.laneDetector)
        for i in range(len(self.threadPool)):
            self.threadPool[i].start()
        # print("Khani():", self.cameraHandler.ROI)

    def start(self):
        self.startThreads()
        while True:
            # self.laneDetector.detectLanes()
            pass
