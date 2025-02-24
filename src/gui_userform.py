import tkinter as tk
from tkinter import messagebox

# Creating a main window
root = tk.Tk()
root.geometry("300x250") #to adjust width and height
root.titlwe("Registration Form")


# Set the window background colour
root.configure(background="yellow")

# Create and configure the form frmae 
form_frame = tk.Frame(root, padx=20, pady=20)
form_frame.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)