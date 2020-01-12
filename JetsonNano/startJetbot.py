import sys
from time import sleep
import khani


def main():
    var = khani.Var()
    KhaniJet = khani.Khani(var)
    print("Khani() object created. wait for 2 seconds...")
    # sleep(2)

    # start threads and kill all threads when Keyboard Interrupt signal (ctrl+c) occurred.
    try:
        KhaniJet.start()
    except (KeyboardInterrupt, SystemExit):
        print('Ctrl+C pressed...main')
        for i in range(len(KhaniJet.threadPool)):
            self.threadPool[i].join()
            self.var.robot.stop()
        sys.exit()


if __name__ == '__main__':
    main()
