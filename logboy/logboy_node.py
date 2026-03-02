import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
import yaml
import importlib
from datetime import datetime
import os
import time

# TODO add buffers to get topic hz for monitoring
# TODO what happens if a topic joins later and record all topics is set?

class LogboyNode(Node):
    
    def __init__(self, storage_path=None, robot_name=None, topics=None, ros_storage_plugin='mcap'):
        """
        Initialize the BagRecorderNode.
        can be configured with parameters or through the configure_recorder() method.
        """
        super().__init__('logboy_node')

        self.storage_path = storage_path
        self.robot_name = robot_name
        self.ros_storage_plugin = ros_storage_plugin
        self.topics = topics # topic to record, if None all topics are recorded

        self.__reset_recording() # initialize recording state
        

    def __get_bag_file_path(self):
        if not self.storage_path:
            raise ValueError("storage_path is not set. Please provide a valid storage path.")
        if not self.robot_name:
            raise ValueError("robot_name is not set. Please provide a valid robot name.")
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return os.path.join(self.storage_path, f"{self.robot_name}_{current_time}")
    
    def __reset_recording(self):
        self._subscriptions = []
        self._writer = None # close the writer by garbage collection
        self._topic_logs = {}

    def __create_writer(self, bag_file_path):
        self._writer = SequentialWriter()
        storage_options = StorageOptions(uri=bag_file_path, storage_id=self.ros_storage_plugin)
        converter_options = ConverterOptions('', '')
        self._writer.open(storage_options, converter_options)

    def __create_subscriptions(self):
        for topic in self.topics:
            topic_name = topic['name']
            topic_type = topic['type']
            
            # dynamically create subscriptions
            try:
                msg_type = self.__get_message_type(topic_type)
            except Exception as e:
                self.get_logger().error(f"failed to subscribe to {topic_name}, unsupported message type: {topic_type}")
                continue

            self._writer.create_topic(TopicMetadata(name=topic_name, type=topic_type, serialization_format='cdr'))
            
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, topic_name=topic_name: self.__callback(msg, topic_name),
                10
            )
            self._subscriptions.append(subscription)
            self.get_logger().info(f"subscribed to topic: {topic_name}")

    def __callback(self, msg, topic_name):

        if topic_name not in list(self._topic_logs.keys()):
            self.get_logger().info(f"first message received on topic: {topic_name}")
            self._topic_logs[topic_name] = {
                'first_msg_time': self.get_clock().now().nanoseconds,
                'last_msg_time': self.get_clock().now().nanoseconds,
                'msg_count': 0
            }
        else:
            self._topic_logs[topic_name]['last_msg_time'] = self.get_clock().now().nanoseconds
            self._topic_logs[topic_name]['msg_count'] += 1
        
        self._writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)  # write the message to the bag file

    def __get_message_type(self, topic_type):
        try:
            # dynamically import the message type
            module_name, class_name = topic_type.replace('/', '.').rsplit('.', 1)
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            raise RuntimeError(f"Failed to import message type '{topic_type}': {e}")
        
    def pre_rec_check(self):
        if self.storage_path is None or self.robot_name is None or self.topics is None:
            return False
    
        if self.storage_path.startswith("~"):
            self.storage_path = os.path.expanduser(self.storage_path)

        return True
        
    def configure_recorder(self, config):
        # validate config for required keys and value types
        mandatory_keys = ['storage_path', 'robot_name', 'topics']
        for key in mandatory_keys:
            if key not in config or not config[key]:
                raise ValueError(f"Mandatory key '{key}' is missing or empty in the configuration.")

        self.storage_path = config.get('storage_path')
        self.robot_name = config.get('robot_name')

        self.topics = config.get('topics')
        if self.topics is None or self.topics == []:
            self.set_rec_topics_all()

        self.ros_storage_plugin = config.get('ros_storage_plugin', self.ros_storage_plugin)
        
    def discover_topics(self):
        discovered_topics = []
        for topic_name, topic_types in self.get_topic_names_and_types():
            discovered_topics.append({'name': topic_name, 'type': topic_types[0]})
        return discovered_topics
    
    def set_rec_topics(self, topics):
        self.topics = topics

    def get_rec_topics(self):
        return self.topics
    
    def set_rec_topics_all(self):
        self.topics = self.discover_topics()
        
    def start_recording(self):
        if not self.pre_rec_check():
            raise ValueError("Recorder is not properly configured. Please provide storage_path, robot_name and topics.")

        try:
            bag_file_path = self.__get_bag_file_path()
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        
        if self._writer is not None:
            self.get_logger().warn("already recording. stopping previous recording first.")
            self.stop_recording()
        
        self.get_logger().info(f"start recording to bag file: {bag_file_path}")

        self.__reset_recording() # reset state for new recording
        self.__create_writer(bag_file_path)
        self.__create_subscriptions()


    def stop_recording(self):
        self.get_logger().info("stopping recording...")
        for subscription in self._subscriptions:
            self.destroy_subscription(subscription)

        self.__reset_recording()

    def pause_recording(self):
        self.get_logger().info("pausing recording...")
        for subscription in self._subscriptions:
            self.destroy_subscription(subscription)
        self._subscriptions = []

    def resume_recording(self):
        if self._subscriptions:
            self.get_logger().warn("there are already active subscriptions. pause first.")
            return
        self.get_logger().info("resuming recording...")
        self.__create_subscriptions()
