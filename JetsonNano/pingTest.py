#!/usr/bin/env python
import serial
from time import sleep

port="/dev/ttyACM0"
rate=9600

s1 = serial.Serial(port, rate)
s1.reset_input_buffer()

while True:
    line = s1.readline().split()
    if len(line) <= 0:
        sleep(0.1)
        break
    distance = int(line[0])
    if distance > 500:
        print("{}".format(distance))
    else:
        print("too close")
    #sleep(0.5)
