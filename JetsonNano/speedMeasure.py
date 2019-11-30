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
        # print(line)
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

def drive(ser, robot, l_speed, r_speed, distance):
    # number of ticks 't' to travel 'x' distance:
    # t = x / 0.0106 (in cm)
    robot.set_motors(l_speed, r_speed)
    cm_per_tick = 0.0107
    ticks = int(distance / cm_per_tick)
    print("going {} needs {} ticks".format(distance, ticks))
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    reset_ticks()
    start = time.time()
    while True:
        r_tick, l_tick = read_serial()
        if r_tick >= ticks or l_tick >= ticks:
            end = time.time()
            elapsed = end - start
            if r_tick >= ticks:
                print("Right wheel reached {}. Goal was {}".format(r_tick, ticks))
                final_ticks = r_tick
            if l_tick >= ticks:
                print("\tLeft wheel reached {}. Goal was {}".format(l_tick, ticks))
                final_ticks = l_tick
            traveled = final_ticks * cm_per_tick
            speed = traveled / elapsed
            print("GoalDistance:{} cm  Traveled:{} cm  Elapsed:{} sec  Speed:{} cm/s"\
                  .format(distance, traveled, elapsed, speed))
            robot.stop()
            # sleep(.5)
            exit()

def main():
    print("import finished.")
    sleep(2)
    robot = Robot()
    # straight: l:0.3 r:0.33
    # straight: l:0.2 r:0.216  7.22 cm/s
    l_speed = 0.3
    r_speed = 0.33
    distance = 100
    t1 = threading.Thread(target=drive, \
                          args=(s1, robot, l_speed, r_speed, distance))
    t1.start()


if __name__ == "__main__":
    main()