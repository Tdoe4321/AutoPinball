from light import Light
from switch import Switch

class Playfield:
    """ Class that will hold all the componenets in our playfield """
    lights = {
        "top" : [Light(), Light(), Light()], 
        "mid" : [Light(), Light(), Light()], 
        "bot" : [Light(), Light(), Light()]
    }

    switches = {
        "top" : [Switch(), Switch(), Switch], 
        "mid" : [Switch(), Switch(), Switch], 
        "bot" : [Switch(), Switch(), Switch]
    }

    score = 0
