import serial
import threading
from time import sleep
# from observerManager import Observer, Event
import khani

class SerialCommunicator (threading.Thread, khani.Khani):
    def __init__(self, port = '/dev/ttyACM0', baudRate = 9600):
        super().__init__()
        self.khani = Khani()
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
