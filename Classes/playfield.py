from light import Light
from switch import Switch

class Playfield:
    def __init__(self, 
                lights = {
                "top" : [Light(), Light(), Light()], 
                "mid" : [Light(), Light(), Light()], 
                "bot" : [Light(), Light(), Light()]
                }, 
                switches = {
                "top" : [Switch(), Switch(), Switch], 
                "mid" : [Switch(), Switch(), Switch], 
                "bot" : [Switch(), Switch(), Switch]
                }, 
                score = 0, bonus = 0, bonus_modifier = 1, switch_list = [-1, -1, -1, -1, -1], mode = ""): 
        """ Class that will hold all the componenets in our playfield """
        self.switches = switches
        self.lights = lights
        self.bonus = bonus
        self.score = score
        self.switch_list = switch_list
        self.bonus_modifier = bonus_modifier
        self.mode = mode
        
    def reset(self):
        self.lights = {
            "top" : [Light(), Light(), Light()], 
            "mid" : [Light(), Light(), Light()], 
            "bot" : [Light(), Light(), Light()]
        }

        self.switches = {
            "top" : [Switch(), Switch(), Switch], 
            "mid" : [Switch(), Switch(), Switch], 
            "bot" : [Switch(), Switch(), Switch]
        }

        # Score calculations
        self.score = 0
        self.bonus = 0
        self.bonus_modifier = 1

        # List of switches hit in order
        self.switch_list = [-1, -1, -1, -1, -1]

        # String of current mode
        self.mode = ""
        
    def setup_pins(self):
        # Setup all the pins for the switches on the playfield
        self.switches["top"][0].pin = 11
        self.switches["mid"][0].pin = 12
        self.switches["bot"][0].pin = 13
        self.switches["bot"][1].pin = 9

        # Setup all the pins for the lights on the playfield
        self.lights["top"][0].pin = 3 # Test Numbers
        self.lights["mid"][0].pin = 4
        self.lights["bot"][0].pin = 5