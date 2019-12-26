import threading
from jetbot import Robot
from time import sleep
import argparse
from imageHandler import ImageHandler
from laneDetector import LaneDetector

class Khani:
    def __init__(self, serial_port, serial_rate):
        self.driver = None
        self.image_handler = None
        self.PWM_manager = None
        self.serial_communicator = None

def capture(im):
    while True:
        im.capture_to_file()
        sleep(0.1)

def markROI(ln):
    while True:
        ln.markROI()
        sleep(0.1)

def main():
    parser = argparse.ArgumentParser(description='Khan')
    parser.add_argument("-p", type=str, help='Port of the serial', default='/dev/ttyACM0')
    parser.add_argument("-b", type=int, help="Baudrate of the serial", default=9600)
    parser.add_argument("-speed", type=float, help="speed in cm/s", default=10)
    args = parser.parse_args()

    # khan = Khan(args.p, args.b)

    robot = Robot()
    imageHandler = ImageHandler(width=640, height=480)
    laneDetector = LaneDetector(imageHandler.camera)
    print("ready to run, wait for 1 second...")
    sleep(1)
    robot.set_motors(0.2,0.3)
    t1 = threading.Thread(target=markROI(), args=(laneDetector,))
    t1.start()
    sleep(10)
    robot.stop()
    # t1.join()

if __name__ == '__main__':
    main()

