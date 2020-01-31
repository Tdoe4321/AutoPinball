class Light:
    """ Class to hold all light outputs on the field """
    def __init__(self, on=False, last_time_on = -1.0, pin = -1, general_light_on_time = 0.1, override_light = None):
        self.on = on
        self.last_time_on = last_time_on
        self.pin = pin
        self.general_light_on_time = general_light_on_time # Typically we want to turn the light on and off when a switch is triggered
        self.override_light = override_light # Sometimes we want to not use the default light off time