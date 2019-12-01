#!/usr/bin/env python
import serial
import threading
from jetbot import Robot
from time import sleep
import time
import queue

robot = Robot()

port = "/dev/ttyACM0"
rate = 115200

s1 = serial.Serial(port, rate)
s1.reset_input_buffer()
print("wait for 2 seconds")
sleep(2)
print("move!")
moving = 0
print("threads: {}".format(threading.active_count()))


def drive(robot, speed_queue):
    speed = 0.0
    while True:
        try:
            speed = speed_queue.get(timeout=1)
            robot.set_motors(speed, speed)
        except queue.Empty:
            pass
        # print("speed:", speed)


speed_queue = queue.Queue()
threading.Thread(target=drive, args=(robot, speed_queue,)).start()

while True:
    print("threads: {}".format(threading.active_count()))
    start = time.time()
    line = s1.readline().split()
    end = time.time()
    print(end - start)
    if len(line) <= 0:
        print("failed to get the right signal from serial")
        sleep(1)
        continue
    distance = int(line[0])
    if distance <= 0:
        continue
        # print("distance = {}".format(distance))
    if distance > 500:
        if moving == 0:
            # print("setting speed....")
            moving = 1
            speed_queue.put(0.3)
    else:
        moving = 0
        speed_queue.put(0.0)
