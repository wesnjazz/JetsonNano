import serial
from time import sleep

def readSerial(s1):
    r = s1.readline()
    try:
        line = r.decode('utf-8').split(",")
    except UnicodeDecodeError:
        line = [999, 999]
        print("UnicodeDecodeError")
        sleep(3)
    if len(line) >= 2:
        # print(line)
        for i in range(0, len(line)):
            try:
                line[i] = int(line[i])
            except ValueError:
                print("ValueError")
                sleep(3)
                pass

        # print("L:{} R:{}".format(line[1], line[0]))
        lastLtick = line[1]
        lastRtick = line[0]
        return line[0], line[1]
    # else:
        # print("\tLast L:{} R:{}".format(lastLtick, lastRtick))
        # return lastLtick, lastRtick

def main():
    s1 = serial.Serial('/dev/ttyACM0', 9600)
    s1.reset_input_buffer()
    s1.reset_output_buffer()
    print(type(s1))
    print(s1)
    if s1 == None:
        print("None Type")
        sleep(3)
        exit()
    while True:
        sleep(0.01)
        try:
            l, r = readSerial(s1)
        except TypeError:
            print("TypeError")
            l, r = 99999, 99999
            sleep(3)
        print("{},{}".format(l,r))

if __name__ == '__main__':
    main()

