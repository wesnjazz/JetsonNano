import serial
import threading
from jetbot import Robot
from time import sleep
import time
import queue
import argparse
import math

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
    global lastLtick, lastRtick
    r = s1.readline()
    line = r.decode('utf-8').split(",")
    if len(line) >= 2:
        # print(line)
        for i in range(0, len(line)): line[i] = int(line[i])
        print("L:{} R:{}".format(line[1], line[0]))
        lastLtick = line[1]
        lastRtick = line[0]
        return line[1], line[0]
    else:
        print("\tLast L:{} R:{}".format(lastLtick, lastRtick))
        return lastLtick, lastRtick

def reset_ticks():
    s1.write(b'r')
    sleep(1.5)

def getAngle(l_ticks, r_ticks, one_cm_per_tick=0.0107, wheel_base=11.3):
    delta_l = one_cm_per_tick * l_ticks
    delta_r = one_cm_per_tick * r_ticks
    delta_s = (delta_r - delta_l) / 2.0
    delta_theta = delta_s / (wheel_base / 2.0)
    delta_theta_in_radian = delta_s / (wheel_base / 2.0)
    delta_theta_in_degree = (180.0 / math.pi) * delta_theta
    return delta_theta_in_degree

def getDeltaXY(current_tick, last_tick, deltaTheta, one_cm_per_tick=0.0107):
    delta_tick = current_tick - last_tick
    return (delta_tick * one_cm_per_tick) * math.cos(deltaTheta), \
            (delta_tick * one_cm_per_tick) * math.sin(deltaTheta)


def getRefPositionXY_atTimeT(t, speed, theta):
    traveledX_atTimeT = t * speed
    xRef = traveledX_atTimeT * math.cos(theta)
    yRef = traveledX_atTimeT * math.sin(theta)
    return xRef, yRef

def getActPositionXY_atTimeT(current_x, delta_theta):
    xAct = current_x * math.cos(delta_theta)
    yAct = current_x * math.sin(delta_theta)
    return xAct, yAct

# def speedToPWM(speed):


def drive(ser, robot, r_speed, l_speed, distance):
    # number of ticks 't' to travel 'x' distance:
    # t = x / 0.0106 (in cm)

    # Drive the robot
    robot.set_motors(r_speed, l_speed)

    # Calculate required ticks to travel given distance
    ticks = int(distance / 0.0107)
    print("going {} needs {} ticks".format(distance, ticks))

    # Flush input/output buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Reset ticks to 0
    reset_ticks()

    '''
    ToDo:
    - calculate error x, y, theta [theta done]
    - map speed to PWM
    
    '''

    one_cm_per_tick = 0.0106
    wheel_base = 11.3
    start_time = time.time()
    l_tick_last = 0
    r_tick_last = 0
    accumulatedX = 0
    accumulatedY = 0
    accumulatedTheta = 0
    last_tick = 0
    while True:
        l_tick, r_tick = read_serial()
        delta_l_tick = l_tick_last - l_tick
        delta_r_tick = r_tick_last - r_tick
        current_tick = (r_tick + l_tick) // 2
        deltaTheta = getAngle(delta_l_tick, delta_r_tick)
        deltaX, deltaY = getDeltaXY(current_tick, last_tick, deltaTheta)
        accumulatedX += deltaX
        accumulatedY += deltaY
        accumulatedTheta += deltaTheta
        delta_t = time.time() - start_time
        # xRef, yRef = getRefPositionXY_atTimeT(delta_t, )

        current_theta = getAngle(l_tick, r_tick)
        # xAct, yAct = getActPositionXY_atTimeT(delta_theta)

        print("Goal:({:.3f},{:.3f},{:.3f}) \
        Act:({:.3f},{:.3f},{:.3f})  RealTheta:{:.3f}".format\
                  (50, 0, 0, accumulatedX, accumulatedY, accumulatedTheta, current_theta))

        # PWM_L, PWM_R = PWM_manager()
        # robot.set_motors(PWM_L, PWM_R)
        l_tick_last = l_tick
        r_tick_last = r_tick
        last_tick = (r_tick_last + l_tick_last) // 2
        if l_tick >= ticks:
            print(" Left wheel reached {}. Goal was {}".format(l_tick, ticks))
            robot.stop()
            # sleep(.5)
            exit()
        if r_tick >= ticks:
            print("Right wheel reached {}. Goal was {}".format(r_tick, ticks))
            robot.stop()
            # sleep(.5)
            exit()


x = 100
y = 0
theta = 0
delta_x = 0
delta_y = 0
delta_theta = 0


def main():
    # parser = argparse.ArgumentParser(description='PD Control Tester')
    # parser.add_argument("-p", type=str, help='Port of the serial', required=True)
    # parser.add_argument("-b", type=int, help="Baudrate of the serial", default=9600)
    # parser.add_argument("-x", type=float, help="x position of Goal in cm", default=100)
    # parser.add_argument("-y", type=float, help="y position of Goal in cm", default=0)
    # parser.add_argument("-theta", type=float, help="theta of Goal", default=0)
    # args = parser.parse_args()

    print("import finished.")
    # sleep(2)
    robot = Robot()
    l_speed = 0.3
    r_speed = 0.3
    distance = 50
    t1 = threading.Thread(target=drive, \
                          args=(s1, robot, l_speed, r_speed, distance))
    t1.start()


if __name__ == "__main__":
    main()