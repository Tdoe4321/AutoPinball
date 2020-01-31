class Switch:
    def __init__(self, on=False, last_time_on=-1.0, pin=-1, num_times_triggered=0):
        """ A class to hold all switches (inputs) to the system """
        self.on = on
        self.last_time_on = last_time_on
        self.pin = pin
        self.num_times_triggered = num_times_triggered