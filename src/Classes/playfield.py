from light import Light
from switch import Switch
from flipper import Flipper

# High Score
from operator import itemgetter
import pickle

from collections import deque

import os

from pprint import pprint

class Playfield:
    def __init__(self, 
                lights = {
                "top" : deque([Light(), Light(), Light()]), 
                "mid" : deque([Light(), Light(), Light(), Light(), Light(), Light(), Light(), Light()]), 
                "bot" : deque([Light(), Light(), Light(), Light()])
                }, 
                switches = {
                "top" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch()]), 
                "mid" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch()]), 
                "bot" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch()])
                },
                coils = deque([Light(), Light(), Light(), Light(), Light()]), start_button = Switch(), autonomy_switch = Switch(),
                score = 0, bonus = 0, bonus_modifier = 1, switch_list = [-1, -1, -1, -1, -1], mode = "", high_scores = None,
                left_flipper = Flipper(flipper_num=1), right_flipper = Flipper(flipper_num=2),
                multiball_counter = 0, autonomy_value = True, checking_highscore = False): 
        """ Class that will hold all the componenets in our playfield """
        self.switches = switches
        self.lights = lights
        self.coils = coils
        self.start_button = start_button
        self.autonomy_switch = autonomy_switch
        self.bonus = bonus
        self.score = score
        self.switch_list = switch_list
        self.bonus_modifier = bonus_modifier
        self.mode = mode
        self.high_scores = high_scores
        self.load_high_scores()
        self.left_flipper = left_flipper
        self.right_flipper = right_flipper
        self.multiball_counter = multiball_counter
        self.autonomy_value = autonomy_value
        self.checking_highscore = checking_highscore

        # All bottom lights should be "Hold" lights
        for light in self.lights["bot"]:
            light.override_light = "Hold"

        # Slingshot debounce time
        self.switches["bot"][2].debounce_time = 0.25
        self.switches["bot"][3].debounce_time = 0.25
        self.switches["bot"][8].debounce_time = 1

        # Set coil activation time
        for coil in self.coils:
            coil.general_light_on_time = 0.1
        
    def reset(self):
        self.lights = {
                "top" : deque([Light(), Light(), Light()]), 
                "mid" : deque([Light(), Light(), Light(), Light(), Light(), Light(), Light(), Light()]), 
                "bot" : deque([Light(), Light(), Light(), Light()])}

        self.switches = {
                "top" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch()]), 
                "mid" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch()]), 
                "bot" : deque([Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch(), Switch()])}

        self.coils = deque([Light(), Light(), Light(), Light(), Light()])

        self.start_button = Switch()
        self.autonomy_switch = Switch()

        # All the bottom lights are "Hold lights"
        for light in self.lights["bot"]:
            light.override_light = "Hold"

        # Slingshot leads have a tendancy to vibrate alot
        # So I'm increasing their debounce time
        self.switches["bot"][2].debounce_time = 0.25
        self.switches["bot"][3].debounce_time = 0.25
        self.switches["bot"][8].debounce_time = 1

        # Set coil activation time
        for coil in self.coils:
            coil.general_light_on_time = 0.1

        # Score calculations
        self.score = 0
        self.bonus = 0
        self.bonus_modifier = 1

        # List of switches hit in order
        # This works as: [oldest --> Newest]
        self.switch_list = [-1, -1, -1, -1, -1, -1]

        # String of current mode
        self.mode = ""

        # List of paired items of 10 High Scores
        self.high_scores = None
        self.load_high_scores()

        # Flippers
        self.left_flipper = Flipper(flipper_num=1)
        self.right_flipper = Flipper(flipper_num=2)

        # Multiball
        self.multiball_counter = 0
        
    def setup_pins(self):
        # Setup all the pins for the switches on the playfield
        # Top row
        self.switches["top"][0].pin = 8
        self.switches["top"][1].pin = 49
        self.switches["top"][2].pin = 50
        self.switches["top"][3].pin = 51
        self.switches["top"][4].pin = 5
        self.switches["top"][5].pin = 15

        # Mid row
        self.switches["mid"][0].pin = 9
        self.switches["mid"][1].pin = 13
        self.switches["mid"][2].pin = 7
        self.switches["mid"][3].pin = 14
        self.switches["mid"][4].pin = 6
        self.switches["mid"][5].pin = 4

        # Bot row
        self.switches["bot"][0].pin = 10
        self.switches["bot"][1].pin = 11
        self.switches["bot"][2].pin = 53
        self.switches["bot"][3].pin = 52
        self.switches["bot"][4].pin = 3
        self.switches["bot"][5].pin = 2
        self.switches["bot"][6].pin = 18
        self.switches["bot"][7].pin = 19
        self.switches["bot"][8].pin = 12

        # Setup all the pins for the lights on the playfield
        # Top row
        self.lights["top"][0].pin = 36
        self.lights["top"][1].pin = 35
        self.lights["top"][2].pin = 34
        
        # Mid row
        self.lights["mid"][0].pin = 31
        self.lights["mid"][1].pin = 32
        self.lights["mid"][2].pin = 27
        self.lights["mid"][3].pin = 28
        self.lights["mid"][4].pin = 29
        self.lights["mid"][5].pin = 26
        self.lights["mid"][6].pin = 25
        self.lights["mid"][7].pin = 24
        
        # Bot row
        self.lights["bot"][0].pin = 33
        self.lights["bot"][1].pin = 30
        self.lights["bot"][2].pin = 23
        self.lights["bot"][3].pin = 22

        # Setup the coils
        self.coils[0].pin = 44
        self.coils[1].pin = 45
        self.coils[2].pin = 46
        self.coils[3].pin = 47
        self.coils[4].pin = 48

        # Start button
        self.start_button.pin = 42
        self.autonomy_switch.pin = 43

    def load_high_scores(self):
        try:
            with open(os.path.dirname(os.path.realpath(__file__)) + '/../../highscores.txt', 'r') as f:
                self.high_scores = pickle.load(f)
        except:
            print("Highscore file not there, generating...")
            self.generate_high_score()
            with open(os.path.dirname(os.path.realpath(__file__)) + '/../../highscores.txt', 'r') as f:
                self.high_scores = pickle.load(f)

    def check_high_score(self, name, score):
        self.high_scores.append((name, score))
        self.high_scores = sorted(self.high_scores, key=itemgetter(1), reverse=True)
        if self.high_scores[10] == (name,score):
            print("Sorry, you didn't make the high score list")
        else:
            print("Congrats! You made the top 10!")
        self.high_scores = self.high_scores[:10]

        pprint(self.high_scores)

        with open(os.path.dirname(os.path.realpath(__file__)) + '/../../highscores.txt', 'w') as f:
            pickle.dump(self.high_scores, f)

    def generate_high_score(self):
        high_scores = [
        ('AAA', 10000),
        ('BBB', 9000),
        ('CCC', 8000),
        ('DDD', 7000),
        ('EEE', 6000),
        ('FFF', 5000),
        ('GGG', 4000),
        ('HHH', 3000),
        ('III', 2000),
        ('JJJ', 1000),
        ] 

        self.high_scores = sorted(high_scores, key=itemgetter(1), reverse=True)[:10]

        with open(os.path.dirname(os.path.realpath(__file__)) + '/../../highscores.txt', 'w') as f:
            pickle.dump(self.high_scores, f)
