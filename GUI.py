import Tkinter as tk
import tkFont
import time


class PinballGUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Welcome to LikeGeeks app")
        self.window.resizable(True, True) 
        self.font = tkFont.Font(size = 50)
        self.lbl = tk.Label(self.window, text="Hello", font=self.font)

    def resize(self, event):
        print("HERE")
        self.lbl = tk.Label(self.window, text="Hello", width=self.window.winfo_width(), height=self.window.winfo_height())
        # self.font = tkFont.Font(size=self.window.winfo_width())
        #self.lbl.config(font=self.font)

if __name__ == "__main__":
    myGUI = PinballGUI()
     

    
    myGUI.lbl.grid(column=0, row=0)

    myGUI.window.bind('<Configure>', myGUI.resize)

    myGUI.window.mainloop()