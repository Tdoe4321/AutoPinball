import Tkinter as tk
import tkFont
import time
import rospy

from std_msgs.msg import Int32


class PinballGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("AutoPinball")
        self.font = tkFont.Font(size = 150)
        self.score_label = tk.Label(self.window, text="0", font=self.font, bg="#FDBB30", fg="#0B1315")

def update_score(data, GUI):
    GUI.score_label['text'] = '{:,}'.format(data.data)

if __name__ == "__main__":
    myGUI = PinballGUI()
    rospy.init_node("Pinball_GUI")
    
    myGUI.score_label.grid(column=0, row=0)

    score_sub = rospy.Subscriber("update_score", Int32, update_score, myGUI)

    myGUI.window.mainloop()