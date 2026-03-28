import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
import importlib
from datetime import datetime
import os

from logboy.logboy_stats import TopicStats, TopicSnapshot


class LogboyNode(Node):

    def __init__(self):
        super().__init__('logboy_node')

        self.storage_path = None
        self.robot_name = None
        self.topics = None

        self._topic_stats: dict[str, TopicStats] = {}
        self._subscriptions = []   # always active (Monitor)
        self._writer = None
        self._recording = False    # controls if __callback writes into bag
        self._bag_path = None

    # ════════════════════════════════════════════
    #  Public – View-Interface
    # ════════════════════════════════════════════

    def get_stats(self) -> list[TopicSnapshot]:
        return [s.snapshot() for s in self._topic_stats.values()]

    def get_bag_path(self) -> str | None:
        return self._bag_path

    # ════════════════════════════════════════════
    #  configuration
    # ════════════════════════════════════════════

    def configure_recorder(self, config):
        mandatory_keys = ['storage_path', 'robot_name', 'topics']
        for key in mandatory_keys:
            if key not in config or not config[key]:
                raise ValueError(f"Mandatory key '{key}' is missing or empty.")

        self.storage_path = config.get('storage_path')
        self.robot_name = config.get('robot_name')
        self.ros_storage_plugin = config.get('ros_storage_plugin', "mcap")
        self._stats_window = config.get('stats_window', 2.0)

        self.topics = config.get('topics')
        if not self.topics:
            self.set_rec_topics_all()

        # create subscriptions for monitoring
        self.__setup_monitor_subscriptions()

    def discover_topics(self):
        return [
            {'name': n, 'type': t[0]}
            for n, t in self.get_topic_names_and_types()
        ]

    def set_rec_topics(self, topics):
        self.topics = topics
        self.__setup_monitor_subscriptions()

    def get_rec_topics(self):
        return self.topics

    def set_rec_topics_all(self):
        self.topics = self.discover_topics()
        self.__setup_monitor_subscriptions()

    # ════════════════════════════════════════════
    #  Recording API
    # ════════════════════════════════════════════

    def start_recording(self):
        if not self.__pre_rec_check():
            raise ValueError("Recorder not configured.")
        if self._recording:
            self.get_logger().warn("Already recording.")
            return

        self._bag_path = self.__get_bag_file_path()
        self.get_logger().info(f"Recording to: {self._bag_path}")
        self.__create_writer(self._bag_path)
        self._recording = True
        for s in self._topic_stats.values():
            s.reset_counts()
            s.set_recording(True)

    def stop_recording(self):
        self.get_logger().info("Stopping recording…")
        self._recording = False
        for s in self._topic_stats.values():
            s.set_recording(False)
        self._writer = None   # close SequentialWriter via GC
        self._bag_path = None

    def pause_recording(self):
        self.get_logger().info("Pausing recording…")
        self._recording = False
        self._recording = False
        for s in self._topic_stats.values():
            s.set_recording(False)

    def resume_recording(self):
        if self._writer is None:
            self.get_logger().warn("No active bag - call start_recording() first.")
            return
        self.get_logger().info("Resuming recording…")
        self._recording = True
        for s in self._topic_stats.values():
            s.set_recording(True)

    # ════════════════════════════════════════════
    #  Private
    # ════════════════════════════════════════════
    
    def __get_bag_file_path(self) -> str:
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return os.path.join(self.storage_path, f"{self.robot_name}_{ts}")

    def __get_message_type(self, topic_type: str):
        module_name, class_name = topic_type.replace('/', '.').rsplit('.', 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)
    
    def __pre_rec_check(self):
        if not (self.storage_path and self.robot_name and self.topics):
            return False
        if self.storage_path.startswith("~"):
            self.storage_path = os.path.expanduser(self.storage_path)
        if not os.path.exists(self.storage_path):
            raise ValueError(f"storage_path does not exist: {self.storage_path}")   
        return True

    def __setup_monitor_subscriptions(self):
        """
        Creates exactly one subscription for each topic.
        Existing subscriptions remain intact (no duplicate subscriptions).
        Called by configure_recorder() and set_rec_topics*().
        """
        existing = set(self._topic_stats.keys())

        for topic in self.topics:
            name: str = topic['name']
            msg_type_str: str = topic['type']
            expected_fps: float = float(topic.get('fps', 0.0))

            if name in existing:
                continue  # subscription already exists

            try:
                msg_cls = self.__get_message_type(msg_type_str)
            except Exception:
                self.get_logger().error(f"Unknown type for {name}: {msg_type_str}")
                continue

            self._topic_stats[name] = TopicStats(
                name=name,
                msg_type=msg_type_str,
                expected_fps=expected_fps,
                window=self._stats_window,
            )

            sub = self.create_subscription(
                msg_cls, name,
                lambda msg, n=name: self.__callback(msg, n),
                10,
            )
            self._subscriptions.append(sub)
            self.get_logger().info(f"Monitoring: {name}  (expected_fps={expected_fps or 'n/a'})")

    def __create_writer(self, bag_path: str):
        self._writer = SequentialWriter()
        self._writer.open(
            StorageOptions(uri=bag_path, storage_id=self.ros_storage_plugin),
            ConverterOptions('', ''),
        )
        for topic in self.topics:
            self._writer.create_topic(TopicMetadata(
                name=topic['name'],
                type=topic['type'],
                serialization_format='cdr',
            ))

    def __callback(self, msg, topic_name: str):
        # always update stats, even if not recording
        self._topic_stats[topic_name].record()

        # only write to bag if recording is active
        if self._recording and self._writer:
            self._writer.write(
                topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds,
            )
