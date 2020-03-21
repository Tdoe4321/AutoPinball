# GUI
import Tkinter as tk
import tkFont

# Testing
import time

# ROS imports
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import String

class PinballGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("AutoPinball")
        self.score_font = tkFont.Font(size = 150)
        self.message_font = tkFont.Font(size = 20)
        self.score_label = tk.Label(self.window, text="SCORE: 0", font=self.score_font, bg="#FDBB30", fg="#0B1315")
        self.message_label = tk.Label(self.window, text="MODE: None", font=self.message_font)

def update_score(data, GUI):
    GUI.score_label['text'] = 'SCORE: ' + '{:,}'.format(data.data)

def update_message(data, GUI):
    GUI.message_label['text'] = data.data

if __name__ == "__main__":
    myGUI = PinballGUI()
    rospy.init_node("Pinball_GUI")
    
    myGUI.score_label.grid(column=0, row=0)
    myGUI.message_label.grid(column=0, row=1, padx=(10,10), pady=(20,20))

    score_sub = rospy.Subscriber("update_score", Int32, update_score, myGUI)
    score_sub = rospy.Subscriber("update_message", String, update_message, myGUI)

    myGUI.window.mainloop()