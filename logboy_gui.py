import os
import subprocess
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

REC_DIR = "/data/recordings"
ROBOT_NAME = "jarvis"


class RecorderApp:
    def __init__(self, root):
        """Initialize the RecorderApp."""
        self.root = root
        self.root.title("logboy")

        # Load images for buttons
        self.record_img = tk.PhotoImage(file="resources/rec-button.png")
        self.record_img_inactive = tk.PhotoImage(
            file="resources/rec-button_inactive_2.png"
        )
        self.pause_img = tk.PhotoImage(file="resources/pause.png")
        self.pause_img_inactive = tk.PhotoImage(file="resources/circular.png")
        self.stop_img = tk.PhotoImage(file="resources/stop-button.png")

        # Create button for record icon
        button_frame = tk.Frame(root)
        button_frame.pack(pady=(10, 5), padx=5)

        self.record_button = tk.Button(
            button_frame,
            image=self.record_img,
            command=self.start_recording,
            width=100,
            height=100,
        )
        self.record_button.grid(row=0, column=0, padx=5, pady=0)

        # Create button for pause icon
        self.pause_button = tk.Button(
            button_frame,
            image=self.pause_img,
            command=self.pause_recording,
            state=tk.DISABLED,
            width=100,
            height=100,
        )
        self.pause_button.grid(row=0, column=1, padx=5, pady=0)

        # Create button for stop icon
        self.stop_button = tk.Button(
            button_frame,
            image=self.stop_img,
            command=self.stop_recording,
            state=tk.DISABLED,
            width=100,
            height=100,
        )
        self.stop_button.grid(row=0, column=2, padx=5, pady=0)

        self.status_label = tk.Label(root, text="Status: Ready", font=("Arial", 12))
        self.status_label.pack(pady=(0, 5))

        self.root.resizable(False, False)

        self.recording = False
        self.paused = False
        self.record_process = None

        # Bind the close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def start_recording(self):
        """Start recording."""
        self.status_label.config(text="Status: Recording")
        self.record_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.pause_button.config(state=tk.NORMAL)
        self.recording = True
        self.paused = False
        self.blink_record_button()

        # Start the ros2 bag record command
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_directory = os.path.join(REC_DIR, f"{ROBOT_NAME}_{current_time}")
        start_time = time.time()
        self.record_process = subprocess.Popen(
            ["ros2", "bag", "record", "-a", "-s", "mcap", "-o", output_directory],
            stdin=subprocess.PIPE,
            text=True,
        )

    def stop_recording(self):
        """Stop recording."""
        self.status_label.config(text="Status: Stopped")
        self.pause_button.config(image=self.pause_img)
        self.record_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.pause_button.config(state=tk.DISABLED)
        self.recording = False
        self.paused = False
        self.record_button.config(image=self.record_img)
        self.pause_button.config(image=self.pause_img)

        # Stop the ros2 bag record command
        if self.record_process:
            self.record_process.terminate()
            self.record_process.wait()
            self.record_process = None

    def pause_recording(self):
        """Pause or resume recording."""
        if not self.paused:
            self.status_label.config(text="Status: Paused")
            self.recording = False
            self.paused = True
            self.blink_pause_button()
            # Pause the ros2 bag record command
            if self.record_process:
                self.record_process.stdin.write(" ")
                self.record_process.stdin.flush()
        else:
            self.status_label.config(text="Status: Recording Resumed")
            self.recording = True
            self.paused = False
            self.blink_record_button()
            # Resume the ros2 bag record command
            if self.record_process:
                self.record_process.stdin.write(" ")
                self.record_process.stdin.flush()
        self.pause_button.config(
            image=self.pause_img_inactive if self.paused else self.pause_img
        )

    def blink_record_button(self):
        """Blink the record button."""
        if self.recording:
            current_image = self.record_button.cget("image")
            next_image = (
                str(self.record_img_inactive)
                if current_image == str(self.record_img)
                else str(self.record_img)
            )
            self.record_button.config(image=next_image)
            self.root.after(1000, self.blink_record_button)  # Change image every 1000ms

    def blink_pause_button(self):
        """Blink the pause button."""
        if not self.recording and self.paused:
            current_image = self.pause_button.cget("image")
            next_image = (
                str(self.pause_img_inactive)
                if current_image == str(self.pause_img)
                else str(self.pause_img)
            )
            self.pause_button.config(image=next_image)
            self.root.after(250, self.blink_pause_button)  # Change image every 250ms

    def on_closing(self):
        """Handle the window close event."""
        if self.record_process:
            self.record_process.terminate()
            self.record_process.wait()
        self.root.destroy()


if __name__ == "__main__":
    # TODO source ROS
    # TODO source ROS workspace

    root = tk.Tk()
    app = RecorderApp(root)
    root.mainloop()
