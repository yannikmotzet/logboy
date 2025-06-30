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

class BagRecorderNode(Node):
    def __init__(self, storage_path=None, robot_name=None, topics=None, ros_storage_plugin='mcap'):
        super().__init__('bag_recorder_node')

        self.storage_path = storage_path
        self.robot_name = robot_name
        self.ros_storage_plugin = ros_storage_plugin
        self.topics = topics # topic to record, if None all topics are recorded
        
        self.subscriptions_rec = [] # list of active subscriptions

        # os.makedirs(self.storage_path, exist_ok=True)

        # if self.topics is None:
        #     self.set_rec_topics_all

        # if config_path is None:
        #     self.declare_parameter('config_path', 'config.yaml')  # path to the YAML configuration file
        #     config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # # load configuration
        # self.get_logger().info(f"loading configuration from: {config_path}")
        # with open(config_path, 'r') as file:
        #     config = yaml.safe_load(file)
        
        # self.storage_path = os.path.expanduser(config.get('storage_path', '/tmp'))
        # os.makedirs(self.storage_path, exist_ok=True)
        # self.robot_name = config.get('robot_name', 'unknown')

        # self.ros_storage_plugin = config.get('ros_storage_plugin', 'mcap')
        # self.topics = config.get('topics', [])
        # self.subscriptions_rec = []

        # if config.get('record_all_topics', False):
        #     self.set_rec_topics_all()

        # self.start_recording()

    def __get_bag_file_path(self):
        if not self.storage_path:
            raise ValueError("storage_path is not set. Please provide a valid storage path.")
        if not self.robot_name:
            raise ValueError("robot_name is not set. Please provide a valid robot name.")
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        return os.path.join(self.storage_path, f"{self.robot_name}_{current_time}")

    def __create_writer(self, bag_file_path):
        self.writer = SequentialWriter()
        storage_options = StorageOptions(uri=bag_file_path, storage_id=self.ros_storage_plugin)
        converter_options = ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

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

            self.writer.create_topic(TopicMetadata(name=topic_name, type=topic_type, serialization_format='cdr'))
            
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, topic_name=topic_name: self.__callback(msg, topic_name),
                10
            )
            self.subscriptions_rec.append(subscription)
            self.get_logger().info(f"subscribed to topic: {topic_name}")

    def __callback(self, msg, topic_name):
        if not hasattr(self, '_received_first_message'):
            self._received_first_message = set()
        
        if topic_name not in self._received_first_message:
            self.get_logger().info(f"first message received on topic: {topic_name}")
            self._received_first_message.add(topic_name)
        
        self.writer.write(topic_name, serialize_message(msg), self.get_clock().now().nanoseconds)  # write the message to the bag file

    def __get_message_type(self, topic_type):
        try:
            # dynamically import the message type
            module_name, class_name = topic_type.replace('/', '.').rsplit('.', 1)
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            raise RuntimeError(f"Failed to import message type '{topic_type}': {e}")
        
    def configure_recorder(self, config):
        self.storage_path = config.get('storage_path', self.storage_path)
        self.robot_name = config.get('robot_name', self.robot_name)
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
        try:
            bag_file_path = self.__get_bag_file_path()
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        
        if not self.topics:
            self.get_logger().warn("no topics configured for recording. using all discovered topics.")
            self.set_rec_topics_all()
        if not self.topics:
            self.get_logger().error("no topics available for recording. cannot start recording.")
            return
        if self.writer is not None:
            self.get_logger().warn("already recording. stopping previous recording first.")
            self.stop_recording()
        
        self.get_logger().info(f"start recording to bag file: {bag_file_path}")
        self.__create_writer(bag_file_path)
        self.__create_subscriptions()

    def stop_recording(self):
        self.get_logger().info("stopping recording...")
        for subscription in self.subscriptions_rec:
            self.destroy_subscription(subscription)
        self.subscriptions_rec = []
        self.writer = None # close the writer by garbage collection

    def pause_recording(self):
        self.get_logger().info("pausing recording...")
        for subscription in self.subscriptions_rec:
            self.destroy_subscription(subscription)
        self.subscriptions_rec = []

    def resume_recording(self):
        if self.subscriptions_rec:
            self.get_logger().warn("there are already active subscriptions. pause first.")
            return
        self.get_logger().info("resuming recording...")
        self.__create_subscriptions()


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    executor = MultiThreadedExecutor(num_threads=None)
    try:
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
