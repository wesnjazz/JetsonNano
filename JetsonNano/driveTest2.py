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

lastRtick = 0
lastLtick = 0

def read_serial():
    global lastRtick, lastLtick
    r = s1.readline()
    line = r.decode('utf-8').split(",")
    if len(line) >= 2:
        print(line)
        for i in range(0, len(line)): line[i] = int(line[i])
        print("R:{} L:{}".format(line[0], line[1]))
        lastRtick = line[0]
        lastLtick = line[1]
        return line[0], line[1]
    else:
        print("\tLast R:{} L:{}".format(lastRtick, lastLtick))
        return lastRtick, lastLtick

def reset_ticks():
    s1.write(b'r')
    sleep(1.5)

def drive(ser, robot, r_speed, l_speed, distance):
    # number of ticks 't' to travel 'x' distance:
    # t = x / 0.0106 (in cm)
    robot.set_motors(r_speed, l_speed)
    ticks = int(distance / 0.0107)
    print("going {} needs {} ticks".format(distance, ticks))
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    reset_ticks()
    # while True:
    #     r_tick, l_tick = read_serial()
    #     if r_tick >= ticks:
    #         print("Right wheel reached {}. Goal was {}".format(r_tick, ticks))
    #         robot.stop()
    #         # sleep(.5)
    #         exit()
    #     if l_tick >= ticks:
    #         print(" Left wheel reached {}. Goal was {}".format(l_tick, ticks))
    #         robot.stop()
    #         # sleep(.5)
    #         exit()

    begin = time.time()
    max = 100
    speedL = l_speed
    speedR = r_speed

    delta = 0.001

    # t = 0.001
    # for i in range(2000):
    #     sleep(t)
    #     robot.set_motors(speedL, speedR)
    #     print('{:.3f}   PWM: {:.3f} {:.3f}'.format(time.time() - begin, speedL, speedR))
    # for i in range(0, max):
    #     speedL -= delta
    #     speedR += delta
    #     sleep(t)
    #     robot.set_motors(speedL, speedR)
    #     print('{:.3f}   PWM: {:.3f} {:.3f}'.format(time.time() - begin, speedL, speedR))
    # for i in range(2000):
    #     sleep(t)
    #     robot.set_motors(speedL, speedR)
    #     print('{:.3f}   PWM: {:.3f} {:.3f}'.format(time.time() - begin, speedL, speedR))
    # for i in range(0, max):
    #     speedL += delta
    #     speedR -= delta
    #     sleep(t)
    #     robot.set_motors(speedL, speedR)
    #     print('{:.3f}   PWM: {:.3f} {:.3f}'.format(time.time() - begin, speedL, speedR))
    # for i in range(2000):
    #     sleep(t)
    #     robot.set_motors(speedL, speedR)
    #     print('{:.3f}   PWM: {:.3f} {:.3f}'.format(time.time() - begin, speedL, speedR))
    # robot.stop()
    #

    from random import seed
    from random import random
    import random
    # seed(1)

    for i in range(100):
        # sleep(0.8)
        speedL = random.uniform(-0.5, 0.5)
        speedR = random.uniform(-0.5, 0.5)
        print('{:3d} {:.3f}   PWM: {:.3f} {:.3f}'.format(i, time.time() - begin, speedL, speedR))
        robot.set_motors(speedL, speedR)
    robot.stop()


def main():
    print("import finished.")
    sleep(2)
    robot = Robot()
    r_speed = 0.25
    l_speed = 0.25
    distance = 50
    t1 = threading.Thread(target=drive, \
                          args=(s1, robot, r_speed, l_speed, distance))
    t1.start()


if __name__ == "__main__":
    main()