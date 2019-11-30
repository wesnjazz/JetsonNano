import serial
import threading
from jetbot import Robot
from time import sleep
import time
import queue

# 1 revolution = 1938 ticks
# wheel circumference = 20.5cm
# 1 tick = 20.5cm / 1938 ticks = 0.0106cm = 0.106mm
# number of ticks 't' to travel 'x' distance:
# t = x / 0.0106 (in cm)

port = "/dev/ttyACM0"
rate = 9600
s1 = serial.Serial(port, rate)
s1.reset_input_buffer()

tickR = 0
tickL = 0
lastUpdateTime = 0
PDHerzTarget = 15
lastRtick = 0
lastLtick = 0

def read_serial():
    # global lastUpdateTime, PDHerzTarget
    # now = time.time()
    #
    # sleepTime = lastUpdateTime + (1.0 / PDHerzTarget) - now
    # if sleepTime > 0:
    #     sleep(sleepTime)
    #
    # now = time.time()
    # # deltaTime = now - self.lastUpdateTime
    #
    # # self.totalTimePassed += deltaTime
    # lastUpdateTime = now

    # read serial, decode bytes into utf-8, split strings by ",", convert all into int

    global lastRtick, lastLtick
    r = s1.readline()
    # print(r)
    line = r.decode('utf-8').split(",")
    if len(line) >= 2:
        # print(line)
        for i in range(0, len(line)): line[i] = int(line[i])
        print("R:{} L:{}".format(line[0], line[1]))
        lastRtick = line[0]
        lastLtick = line[1]
        return line[0], line[1]
    else:
        print("Last R:{} L:{}".format(lastRtick, lastLtick))
        # sleep(1)
        return lastRtick, lastLtick

def reset_ticks():
    s1.write(b'r')
    sleep(1)

def drive(ser, robot, r_speed, l_speed, distance):
    # number of ticks 't' to travel 'x' distance:
    # t = x / 0.0106 (in cm)
    robot.set_motors(r_speed, l_speed)
    ticks = int(distance / 0.0106)
    print("going {} needs {} ticks".format(distance, ticks))
    # sleep(2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    reset_ticks()
    while True:
        r_tick, l_tick = read_serial()
        if r_tick >= ticks:
            print("Right wheel reached {}. Goal was {}".format(r_tick, ticks))
            robot.stop()
            sleep(.5)
            # exit()
        if l_tick >= ticks:
            print(" Left wheel reached {}. Goal was {}".format(l_tick, ticks))
            robot.stop()
            sleep(.5)
            # exit()
# def get_ticks():
#     data = read_serial()
#     for i in range(0, len(data)): data[i] = int(data[i])
#     return data[0], data[1]


def main():
    print("import finished.")
    # reset_ticks()
    sleep(2)
    robot = Robot()
    r_speed = 0.3
    l_speed = 0.3
    distance = 20.5
    t1 = threading.Thread(target=drive, \
                          args=(s1, robot, r_speed, l_speed, distance))
    t1.start()

    # t2 = threading.Thread(target=get_ticks, \
    #                       args=(r_tick_queue,l_tick_queue))
    # t2.start()

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