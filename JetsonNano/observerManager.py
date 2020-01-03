from time import sleep

class Observer:
    # Store all observers in a higher level variable
    _observers = []
    def __init__(self):
        # Add itself to the observers list
        self._observers.append(self)
        # Store all observable events for each observer independently
        self._observables = {}
        print(self._observers)
        print(self._observables)
    def observe(self, event_name, callback):
        # Assign callback function when the event triggered
        self._observables[event_name] = callback

class Event:
    def __init__(self, event_name, data, autofire = True):
        # Event name and the argument data
        self.event_name = event_name
        self.data = data
        if autofire:
            self.fire()
    def fire(self):
        # Search all observer instances
        for observer in Observer._observers:
            # Search if the event exists in any observer's _observables{}
            if self.event_name in observer._observables:
                # Fire! with the argument data
                observer._observables[self.event_name](self.data)


# class A:
#     def __init__(self, value):
#         self.value = value
#     def add(self, x):
#         self.value += x
#     def event(self):
#         Event('Changed', self.value)
#
#
# class B(Observer):
#     def __init__(self):
#         Observer.__init__(self)
#         self.value = 0
#     def follow(self, X):
#         print(type(X))
#         self.value = X.value * 2
#
# class C(Observer):
#     def __init__(self):
#         Observer.__init__(self)
#         self.value = 0
#     def follow(self, X):
#         self.value = X.value * 3
#
#
# value = 5
# a = A(1)
# b = B()
# c = C()
#
# b.observe('Changed', b.follow)
# c.observe('Changed', c.follow)
# print(b, b.value)
# print(c, c.value)
#
# while True:
#     a = A(value)
#     print(value)
#     Event('Changed', a)
#     print(b, b.value)
#     print(c, c.value)
#     print()
#     value += 5
#     sleep(2)