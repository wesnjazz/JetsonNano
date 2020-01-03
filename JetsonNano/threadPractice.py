import threading

class Khani (threading.Thread):
    def __init__(self):
        super().__init__()
        self.value = 0

    def run(self):
        while True:
            # print("Khani(): ", self.value)
            self.value += 1
            print(threading.current_thread())

class TToi(threading.Thread):
    def __init__(self):
        super().__init__()
        self.value = 0

    def run(self):
        while True:
            # print("TToi(): ", self.value)
            self.value -= 1
            print(threading.current_thread())

t1 = Khani()
t2 = TToi()

t1.start()
t2.start()

