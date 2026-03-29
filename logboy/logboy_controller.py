import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
import threading
import time
from logboy.logboy_node import LogboyNode

class LogboyController:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = LogboyNode()
        self._record_start_time: float | None = None
        self._paused_duration: float = 0.0
        self._pause_start_time: float | None = None
        self._max_duration: float | None = None
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

    def set_max_duration(self, seconds: float | None):
        self._max_duration = seconds

    def get_max_duration(self) -> float | None:
        return self._max_duration

    def check_max_duration(self):
        if self._max_duration is None:
            return
        elapsed = self.get_elapsed()
        if elapsed is not None and elapsed >= self._max_duration:
            self.stop_recording()

    def start_recording(self):
        self.node.start_recording()
        self._record_start_time = time.monotonic()
        self._paused_duration = 0.0
        self._pause_start_time = None

    def stop_recording(self):
        self.node.stop_recording()
        self._record_start_time = None
        self._paused_duration = 0.0
        self._pause_start_time = None

    def pause_recording(self):
        self.node.pause_recording()
        self._pause_start_time = time.monotonic()

    def resume_recording(self):
        self.node.resume_recording()
        if self._pause_start_time is not None:
            self._paused_duration += time.monotonic() - self._pause_start_time
            self._pause_start_time = None

    def get_elapsed(self) -> float | None:
        if self._record_start_time is None:
            return None
        elapsed = time.monotonic() - self._record_start_time - self._paused_duration
        if self._pause_start_time is not None:
            elapsed -= time.monotonic() - self._pause_start_time
        return elapsed

    def get_topics(self):
        return self.node.get_rec_topics()
    
    def get_stats(self):
        return self.node.get_stats()

    def get_bag_path(self):
        return self.node.get_bag_path()

    def is_paused(self) -> bool:
        return self.node.is_paused()