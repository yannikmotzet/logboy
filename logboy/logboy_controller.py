import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import threading
from logboy.logboy_node import LogboyNode

class LogboyController:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = LogboyNode()
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def shutdown(self):
        self._executor.shutdown()
        self._spin_thread.join()
        if rclpy.ok():
            rclpy.shutdown()

    def configure_monitor(self):
        self.node.configure_monitor()

    def configure_recorder(self, config):
        self.node.configure_recorder(config)

    def set_topics(self, topics):
        self.node.set_rec_topics(topics)

    def start_recording(self):
        self.node.start_recording()

    def stop_recording(self):
        self.node.stop_recording()

    def pause_recording(self):
        self.node.pause_recording()

    def resume_recording(self):
        self.node.resume_recording()

    def get_topics(self):
        return self.node.get_rec_topics()
    
    def get_stats(self):
        return self.node.get_stats()

    def get_bag_path(self):
        return self.node.get_bag_path()