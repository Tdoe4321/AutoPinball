#from Tkinter import font
import tkFont as font
import Tkinter as tk


class SimpleGUIExample:
    def __init__(self, master):
        self.master = master
        self.master.title("A simple Label")
        self.master.bind('<Configure>', self.resize)

        self.label_font = font.Font(self.master, family='Arial', size=12, weight='bold')

        self.label = tk.Label(self.master, text="Simple Label Resizing!")
        self.label.config(font=self.label_font)
        self.label.pack(fill=tk.BOTH, expand=tk.YES)

        self.close_button = tk.Button(self.master, text="Close", command=master.quit)
        self.close_button.pack()

    def resize(self, event):
        height = self.label.winfo_height()
        width = self.label.winfo_width()
        height = height // 2
        print('height %s' % height)
        print('width %s' % width)
        if height < 10 or width < 200:
            height = 10
        elif width < 400 and height > 20:
            height = 20
        elif width < 600 and height > 30:
            height = 30
        else:
            height = 40
        print('height %s' % height)

        self.label_font['size'] = height
        print(self.label_font.actual())


root = tk.Tk()
simple_gui = SimpleGUIExample(root)
root.mainloop()