#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import cv2
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
import os
from itertools import count

# Code adapted from https://stackoverflow.com/questions/40619731/display-animated-gif-in-tkinter-python-3-5
class GifLabel(tk.Label):
    """a label that displays images, and plays them if they are gifs"""
    def load(self, im):

        self.configure(bg="black")
        width = self.winfo_screenwidth()
        height = self.winfo_screenheight() # can resize each frame, but costs computation time
        
        if isinstance(im, str):
            im = Image.open(im)
        self.loc = 0
        self.frames = []
        self.delays = []

        try:
            for i in count(1):
                self.frames.append(
                    ImageTk.PhotoImage(im.resize((width,height)))
                    )
                try:
                    self.delays.append(im.info['duration'])
                except:
                    self.delays.append(100)
                im.seek(i)
        except EOFError:
            pass

        # try:
        #     self.delay = im.info['duration']
        # except:
        #     self.delay = 100

        if len(self.frames) == 1:
            self.config(image=self.frames[0])
        else:
            self.next_frame()

    def unload(self):
        self.config(image="")
        self.frames = None
        if hasattr(self, 'after_call'):
            self.after_cancel(self.after_call)

    def next_frame(self):
        if self.frames:
            self.loc += 1
            self.loc %= len(self.frames)
            self.config(image=self.frames[self.loc])
            self.after_call = self.after(self.delays[self.loc], self.next_frame)

class FaceSwitcher:
    def __init__(self):
        # Load the GIFs
        home_dir = os.path.expanduser("~")
        self.default_face_path = os.path.join(home_dir, 'Pictures/default_face.gif')
        self.thinking_face_path = os.path.join(home_dir, 'Pictures/thinking_face.gif')
        self.talking_face_path = os.path.join(home_dir, 'Pictures/talking_face.gif')
        self.current_image_path = self.default_face_path

        # Set up Tkinter
        self.root = tk.Tk()
        self.root.configure(bg="black")
        self.root.attributes('-fullscreen', True)  # Fullscreen mode
        self.root.bind("<Escape>", self.exit_fullscreen)
        # self.root.bind("<Escape>", lambda e: self.root.quit())  # Exit on Escape key
        self.root.attributes('-topmost', True)


        # Create a label to display the images
        self.label = GifLabel(self.root)
        self.label.pack(expand=True)
        self.label.load(self.current_image_path)

        # Set up ROS node
        rospy.init_node('face_switcher_node')

        # Create service servers
        rospy.Service('/default_face', Empty, self.show_default_face)
        rospy.Service('/thinking_face', Empty, self.show_thinking_face)
        rospy.Service('/talking_face', Empty, self.show_talking_face)

    def exit_fullscreen(self, event=None):
        """Exit fullscreen mode."""
        self.root.attributes('-fullscreen', False)

    def show_default_face(self, req):
        if self.current_image_path == self.default_face_path:
            return []
        rospy.loginfo("Switching to default face")
        self.current_image_path = self.default_face_path
        self.update_display()
        return []

    def show_thinking_face(self, req):
        if self.current_image_path == self.thinking_face_path:
            return []
        rospy.loginfo("Switching to thinking face")
        self.current_image_path = self.thinking_face_path
        self.update_display()
        return []
    
    def show_talking_face(self, req):
        if self.current_image_path == self.talking_face_path:
            return []
        rospy.loginfo("Switching to talking face")
        self.current_image_path = self.talking_face_path
        self.update_display()
        return []

    def update_display(self):
        self.label.unload()
        self.label.load(self.current_image_path)

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    face_switcher = FaceSwitcher()
    face_switcher.run()
