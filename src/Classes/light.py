class Light:
    """ Class to hold all light outputs on the field """
    def __init__(self, on=False, last_time_on = -1.0, pin = -1, general_light_on_time = 0.2, override_light = "None", curr_number_blink = 0, blink_start_time = -1):
        self.on = on
        self.last_time_on = last_time_on
        self.pin = pin
        self.general_light_on_time = general_light_on_time # Typically we want to turn the light on and off when a switch is triggered
        self.override_light = override_light # Sometimes we want to not use the default light off time
        self.curr_number_blink = curr_number_blink
        self.blink_start_time = blink_start_time