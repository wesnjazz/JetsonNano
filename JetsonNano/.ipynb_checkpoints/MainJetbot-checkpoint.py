import serial
import threading
# from jetbot import Robot
from time import sleep
import time
import queue
import argparse

class Khan:
    def __init__(self, serial_port, serial_rate):
        self.driver = None
        self.image_handler = None
        self.PWM_manager = None
        self.serial_communicator = None


def main():
    parser = argparse.ArgumentParser(description='Khan')
    parser.add_argument("-p", type=str, help='Port of the serial', required=True)
    parser.add_argument("-b", type=int, help="Baudrate of the serial", default=9600)
    parser.add_argument("-speed", type=float, help="speed in cm/s", default=10)
    args = parser.parse_args()

    khan = Khan(args.p, args.b)

if __name__ == '__main__':
    main()

