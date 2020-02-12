import Tkinter as tk

if __name__ == "__main__":
    window = tk.Tk() 
    window.title("Welcome to LikeGeeks app")
    window.resizable(True, True) 
    lbl = tk.Label(window, text="Hello", font=("Arial Bold", 50))
    #lbl.grid(column=0, row=0)
    window.mainloop()