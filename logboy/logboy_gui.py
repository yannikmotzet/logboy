import tkinter as tk
import os

class BagRecorderGUI:
    def __init__(self, root, controller):
        """Initialize the RecorderApp."""
        self.root = root
        self.root.title("logboy")
        self.controller = controller

        # Get the first path from AMENT_PREFIX_PATH or use a default fallback
        ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
        paths = ament_prefix_path.split(os.pathsep)

        if not paths or not paths[0]:
            raise ValueError("AMENT_PREFIX_PATH is not set or invalid.")

        base_path = paths[0]

        # Construct the full path to the assets directory
        package_share_directory = os.path.join(base_path, 'share', 'logboy', 'assets')

        self.record_img = self.__load_image(os.path.join(package_share_directory, "rec-button.png"))
        self.record_img_inactive = self.__load_image(os.path.join(package_share_directory, "rec-button_inactive_2.png"))
        self.pause_img = self.__load_image(os.path.join(package_share_directory, "pause.png"))
        self.pause_img_inactive = self.__load_image(os.path.join(package_share_directory, "circular.png"))
        self.stop_img = self.__load_image(os.path.join(package_share_directory, "stop-button.png"))

        # create button for record icon
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

        # create button for pause icon
        self.pause_button = tk.Button(
            button_frame,
            image=self.pause_img,
            command=self.pause_recording,
            state=tk.DISABLED,
            width=100,
            height=100,
        )
        self.pause_button.grid(row=0, column=1, padx=5, pady=0)

        # create button for stop icon
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

        # bind the close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    
    def __load_image(self, file_path, fallback_color="gray", size=(100, 100)):
        """Load an image or create a dummy image if the file doesn't exist."""
        if os.path.exists(file_path):
            return tk.PhotoImage(file=file_path)
        else:
            print(f"Image not found: {file_path}. Using dummy image.")
            dummy_image = tk.PhotoImage(width=size[0], height=size[1])
            dummy_image.put(fallback_color, to=(0, 0, size[0], size[1]))
            return dummy_image


    def __blink_record_button(self):
        """Blink the record button."""
        if self.recording:
            current_image = self.record_button.cget("image")
            next_image = (
                str(self.record_img_inactive)
                if current_image == str(self.record_img)
                else str(self.record_img)
            )
            self.record_button.config(image=next_image)
            self.root.after(1000, self.__blink_record_button)  # Change image every 1000ms


    def __blink_pause_button(self):
        """Blink the pause button."""
        if not self.recording and self.paused:
            current_image = self.pause_button.cget("image")
            next_image = (
                str(self.pause_img_inactive)
                if current_image == str(self.pause_img)
                else str(self.pause_img)
            )
            self.pause_button.config(image=next_image)
            self.root.after(250, self.__blink_pause_button)  # Change image every 250ms

    
    def configure_recorder(self):
        return {"storage_path": "/tmp", "robot_name": "logboy", "ros_storage_plugin": "mcap"}

    def start_recording(self):
        self.controller.configure_recorder(self.configure_recorder())
        self.controller.start_recording()

        self.status_label.config(text="Status: Recording")
        self.record_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.pause_button.config(state=tk.NORMAL)
        self.recording = True
        self.paused = False
        self.__blink_record_button()


    def stop_recording(self):
        self.controller.stop_recording()

        self.status_label.config(text="Status: Stopped")
        self.pause_button.config(image=self.pause_img)
        self.record_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.pause_button.config(state=tk.DISABLED)
        self.recording = False
        self.paused = False
        self.record_button.config(image=self.record_img)
        self.pause_button.config(image=self.pause_img)


    def pause_recording(self):
        """Pause or resume recording."""
        if not self.paused:
            self.controller.pause_recording()

            self.status_label.config(text="Status: Paused")
            self.recording = False
            self.paused = True
            self.__blink_pause_button()
        else:
            self.controller.resume_recording()

            self.status_label.config(text="Status: Recording Resumed")
            self.recording = True
            self.paused = False
            self.__blink_record_button()

        self.pause_button.config(
            image=self.pause_img_inactive if self.paused else self.pause_img
        )


    def on_closing(self):
        """Handle the window close event."""
        if self.record_process:
            self.record_process.terminate()
            self.record_process.wait()
        self.root.destroy()
