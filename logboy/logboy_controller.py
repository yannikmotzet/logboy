from logboy.logboy_node import BagRecorderNode

class BagRecorderController:
    def __init__(self):
        self.node = BagRecorderNode()

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