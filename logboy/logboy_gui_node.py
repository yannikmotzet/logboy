import rclpy
from rclpy.node import Node
import tkinter as tk
import sys
import threading
from logboy.logboy_controller import BagRecorderController
from logboy.logboy_gui import BagRecorderGUI
import os

class BagRecorderGUINode(Node):
    def __init__(self):
        super().__init__('bag_recorder_gui_node')

        # Initialize the ROS controller with parameters
        self.controller = BagRecorderController()



def ros_spin_thread(node):
    """Spin the ROS node in a separate thread."""
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)

    # Create the ROS node
    gui_node = BagRecorderGUINode()

    # Start the ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(gui_node,), daemon=True)
    ros_thread.start()


    # Start the GUI in the main thread
    root = tk.Tk()
    BagRecorderGUI(root, gui_node.controller)
    root.mainloop()

    
if __name__ == "__main__":
    main()