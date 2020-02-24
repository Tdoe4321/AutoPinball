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
                "mid" : deque([Light(), Light(), Light()]), 
                "bot" : deque([Light(), Light(), Light()])
                }, 
                switches = {
                "top" : deque([Switch(), Switch(), Switch]), 
                "mid" : deque([Switch(), Switch(), Switch]), 
                "bot" : deque([Switch(), Switch(), Switch])
                }, 
                score = 0, bonus = 0, bonus_modifier = 1, switch_list = [-1, -1, -1, -1, -1], mode = "", high_scores = None,
                left_flipper = Flipper(flipper_num=1), right_flipper = Flipper(flipper_num=2)): 
        """ Class that will hold all the componenets in our playfield """
        self.switches = switches
        self.lights = lights
        self.bonus = bonus
        self.score = score
        self.switch_list = switch_list
        self.bonus_modifier = bonus_modifier
        self.mode = mode
        self.high_scores = high_scores
        self.load_high_scores()
        self.left_flipper = left_flipper
        self.right_flipper = right_flipper
        
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

        # List of paired items of 10 High Scores
        self.high_scores = None
        self.load_high_scores()

        # Flippers
        self.left_flipper = Flipper(flipper_num=1)
        self.right_flipper = Flipper(flipper_num=2)
        
    def setup_pins(self):
        # Setup all the pins for the switches on the playfield
        self.switches["top"][0].pin = 34
        self.switches["top"][1].pin = 46
        self.switches["mid"][0].pin = 48
        self.switches["bot"][0].pin = 33
        self.switches["bot"][1].pin = 49

        # Setup all the pins for the lights on the playfield
        self.lights["top"][0].pin = 22 # Test Numbers
        self.lights["mid"][0].pin = 99
        self.lights["bot"][0].pin = 99

    def load_high_scores(self):
        try:
            with open(os.path.dirname(os.path.realpath(__file__)) + '/../highscores.txt', 'r') as f:
                self.high_scores = pickle.load(f)
        except:
            print("Highscore file not there, generating...")
            self.generate_high_score()
            with open(os.path.dirname(os.path.realpath(__file__)) + '/../highscores.txt', 'r') as f:
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

        with open(os.path.dirname(os.path.realpath(__file__)) + '/../highscores.txt', 'w') as f:
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

        with open(os.path.dirname(os.path.realpath(__file__)) + '/../highscores.txt', 'w') as f:
            pickle.dump(self.high_scores, f)
