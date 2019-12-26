import threading
import sys
from jetbot import Robot
from time import sleep

class RobotDriver:
    def __init__(self, khani):
        self.khani = khani
        self.motorSpeedLeft = 0
        self.motorSpeedRight = 0

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
        sleep(2)
        goalTicksLeft = self.khani.tickLeft + requiredTicks
        goalTicksRight = self.khani.tickRight + requiredTicks
        self.khani.robot.set_motors(motorSpeedLeft, motorSpeedRight)
        while self.khani.tickLeft <= goalTicksLeft and self.khani.tickRight <= goalTicksRight:
            pass
        self.khani.robot.stop()

    def turnXLaps(self, counterClockwise = True):
        if counterClockwise == True:
            self.khani.robot.set_motors(0.3, 0.6)
        sleep(10)
        self.khani.robot.stop()


    def calculateTicksForXCm(self, x):
        return x // self.cm_per_tick