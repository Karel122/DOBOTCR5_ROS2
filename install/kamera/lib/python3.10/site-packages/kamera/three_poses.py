import pyrealsense2 as rs

import numpy as np

import cv2

from tkinter import Tk, filedialog, Button, Label, Frame, Canvas, PhotoImage

import os

from PIL import Image, ImageTk

 

class RealSenseApp:

    def __init__(self):

        self.pipeline = rs.pipeline()

        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

 

        # Start the RealSense pipeline

        self.pipeline.start(self.config)

 

        # Initialize variables

        self.frame = None

        self.saved_image = None

 

        # Initialize Tkinter window

        self.root = Tk()

        self.root.title("Intel RealSense Image Capture")

 

        # Create GUI components

        self.create_gui()

        self.update_frame()

 

    def create_gui(self):

        # Create a canvas for displaying the image

        self.canvas = Canvas(self.root, width=1280, height=720)

        self.canvas.pack()

 

        # Create a button to save the image

        self.save_button = Button(self.root, text="Save Image", command=self.save_image)

        self.save_button.pack()

 

    def update_frame(self):

        # Fetch the next color frame from the RealSense pipeline

        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:

            self.root.after(10, self.update_frame)

            return

 

        # Convert to numpy array

        self.frame = np.asanyarray(color_frame.get_data())

 

        # Convert to an image format compatible with Tkinter

        image = Image.fromarray(self.frame)

        photo = ImageTk.PhotoImage(image=image)

 

        # Update the canvas with the new frame

        self.canvas.create_image(0, 0, anchor="nw", image=photo)

        self.canvas.image = photo

 

        # Schedule the next frame update

        self.root.after(10, self.update_frame)

 

    def save_image(self):

        if self.frame is not None:

            # Open file dialog to select save location

            # file_path = filedialog.asksaveasfilename(

            #     defaultextension=".jpg",

            #     filetypes=[("JPEG files", "*.jpg"),("PNG files", "*.png"),("All files", "*.*")],

            #     title="Save Image")

            current_dir = os.getcwd()

            current_dir = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/"

            relative_folder = ""

            file_path = os.path.join(current_dir, relative_folder)

            num = len([entry for entry in os.listdir(file_path) if os.path.isfile(os.path.join(file_path, entry))])

            image_name = f"image10{num}.jpg"

            file_path = os.path.join(file_path,image_name)

            print(file_path)

            if file_path:

                # Attempt to save the image

                success = cv2.imwrite(file_path, self.frame)

            if success:

                print(f"Image successfully saved at: {file_path}")

            else:

                print("Failed to save the image. Please check the file path and permissions.")

 

    def cleanup(self):

        # Stop the pipeline

        self.pipeline.stop()

 

    def run(self):

        try:

            self.root.mainloop()

        except KeyboardInterrupt:

            self.cleanup()

        finally:

            self.cleanup()

 

if __name__ == "__main__":

    app = RealSenseApp()

    app.run()