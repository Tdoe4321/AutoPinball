class Flipper:
    """ Class to hold flipper outputs on the field """
    def __init__(self, on=False, last_time_on = -1.0, flipper_num = -999, general_flipper_on_time=0.3):
        self.on = on
        self.last_time_on = last_time_on
        self.flipper_num = flipper_num
        self.general_flipper_on_time = general_flipper_on_time