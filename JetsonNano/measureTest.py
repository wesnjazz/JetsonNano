import serial
import threading
from jetbot import Robot
from time import sleep

# 1 revolution = 1938 ticks
# wheel circumference = 20.5cm
# 1 tick = 20.5cm / 1938 ticks = 0.0106cm = 0.106mm
# number of ticks 't' to travel 'x' distance:
# t = x / 0.0106 (in cm)

port = "/dev/ttyACM0"
rate = 57600
s1 = serial.Serial(port, rate)
s1.reset_input_buffer()

tickR = 0
tickL = 0

def read_serial():
    global tickR, tickL
    count = 0
    while True:
        count += 1
        line = s1.readline().decode('utf-8').split()
        if line[0] == "R":
            tickR = line[2]
        elif line[0] == "L":
            tickL = line[2]
        print("read_serial count:{}  output:{}  tickR:{} tickL:{}"\
              .format(count, line, tickR, tickL))

def main():

    print("import finished.")
    t1 = threading.Thread(name="ZZZ", target=read_serial)
    t1.start()
    # t1 = threading.Thread(name="ZZZ", target=read_serial2, args=(ser))
    # t1.start()
    # t2 = threading.Thread(name="XXX", target=print_line)
    # t2.start()

    # sleep(1)
    # robot = Robot()
    # robot.forward(0.3)
    # sleep(1)
    # robot.stop()
    #
    # print("wait for 3 seconds...")
    # sleep(3)
    # s1.write(b'r')
    # robot.backward(0.3)
    # sleep(1)
    # robot.stop()

if __name__ == "__main__":
    main()