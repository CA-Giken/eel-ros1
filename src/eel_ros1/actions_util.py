import eel
import tkinter as tk
from tkinter import filedialog

@eel.expose
def open_filebrowser() -> str:
    """
    Opens a file browser dialog and returns the selected file path

    Returns:
    str: The selected file path
    """
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename()
    return file_path