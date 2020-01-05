import sys
from time import sleep
import khani


def main():
    KhaniJet = khani.Khani()
    print("Khani() object created. wait for 2 seconds...")
    sleep(2)

    # start threads and kill all threads when Keyboard Interrupt signal (ctrl+c) occurred.
    try:
        KhaniJet.start()
    except (KeyboardInterrupt, SystemExit):
        for i in range(len(KhaniJet.threadPool)):
            self.threadPool[i].join()
            sys.exit()

    # khani.robotDriver.driveXcm(100,0.0,0.0)
    # khani.robotDriver.driveXcm(30,0.3,0.33)
    # khani.robotDriver.turnXLaps()
    # while True:
    #     print("{},{}".format(khani.tickLeft, khani.tickRight))



if __name__ == '__main__':
    main()
