#!/usr/bin/env python

# GUI
import Tkinter as tk
import tkFont

# Testing
import time

# ROS imports
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import String

import signal, sys

class PinballGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("AutoPinball")
        self.score_font = tkFont.Font(size = 100)
        self.message_font = tkFont.Font(size = 20)
        self.score_label = tk.Label(self.window, text="SCORE: 0\nBONUS: 0", font=self.score_font, bg="#FDBB30", fg="#0B1315")
        self.message_label = tk.Label(self.window, text="MODE: None", font=self.message_font)

        self.score = 0
        self.bonus = 0

    def score_update_string(self, value):
        return 'SCORE: ' + '{:,}\nBONUS: {:,}'.format(value, self.bonus)

    def bonus_update_string(self, value):
        return 'SCORE: ' + '{:,}\nBONUS: {:,}'.format(self.score, value)

def update_score(data, GUI):
    GUI.score = data.data
    GUI.score_label['text'] = GUI.score_update_string(data.data)

def update_message(data, GUI):
    GUI.message_label['text'] = data.data

def update_bonus(data, GUI):
    GUI.bonus = data.data
    GUI.score_label['text'] = GUI.bonus_update_string(data.data)

def signal_handler(sig, frame):
    myGUI.window.destroy()
    sys.exit()

myGUI = PinballGUI()

if __name__ == "__main__":
    rospy.init_node("Pinball_GUI")

    signal.signal(signal.SIGINT, signal_handler)
    
    myGUI.score_label.grid(column=0, row=0)
    myGUI.message_label.grid(column=0, row=1, padx=(10,10), pady=(20,20))

    score_sub = rospy.Subscriber("update_score", Int32, update_score, myGUI)
    score_sub = rospy.Subscriber("update_message", String, update_message, myGUI)
    score_sub = rospy.Subscriber("update_bonus", Int32, update_bonus, myGUI)

    myGUI.window.mainloop()
