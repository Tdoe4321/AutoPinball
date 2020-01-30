class Light:
    """ Class to hold all light outputs on the field """
    on = False
    last_time_on = -1.0
    pin = -1
    general_light_on_time = 0.1 # Typically we want to turn the light on and off when a switch is triggered
    override_light = None # Sometimes we want to not use the default light off time