class Switch:
    """ A class to hold all switches (inputs) to the system """
    on = False
    last_time_on = -1.0
    pin = -1
    num_times_triggered = 0