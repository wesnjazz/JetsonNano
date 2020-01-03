
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
                print(self.event_name, observer._observables)
                observer._observables[self.event_name](self.data)
