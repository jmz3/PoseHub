import zmq
import time
import sys
import threading


class ZMQManager:
    def __init__(self, sub_ip, sub_port, pub_port, sub_topic, pub_topic, sensor_name):
        self.sub_ip = sub_ip
        self.sub_port = sub_port
        self.pub_port = pub_port
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.sensor_name = sensor_name
        self.connected = False
        self.pub_messages = {
            topic: f"{topic} Waiting for message..." for topic in self.pub_topic
        }
        self.sub_poses = {}

    def initialize_publisher(self):
        context = zmq.Context()
        publisher = context.socket(zmq.PUB)
        publisher.bind(f"tcp://*:{self.pub_port}")
        time.sleep(1)
        return context, publisher

    def initialize_subscriber(self):
        context = zmq.Context()
        subscriber = context.socket(zmq.SUB)
        subscriber.connect(f"tcp://{self.sub_ip}:{self.sub_port}")

        subscriber.setsockopt(zmq.RCVTIMEO, 1000)

        if isinstance(self.sub_topic, str):
            self.sub_topic = self.sub_topic.encode()

        for t in self.sub_topic:
            subscriber.setsockopt(zmq.SUBSCRIBE, t)

        return context, subscriber

    @staticmethod
    def process_message(topic, message):
        return topic + f" {message}"

    def update_SubPoses(self, topic, msg):
        self.sub_poses[topic] = msg

    def update_PubMsgs(self, topic, msg):
        self.pub_messages[topic] = msg

    def subscriber_thread(self):
        sub_context, sub_socket = self.initialize_subscriber()
        while True:
            if self.connected:
                try:
                    topic, message = sub_socket.recv_multipart()
                    self.update_SubPoses(topic.decode("utf-8"), message.decode("utf-8"))
                    print(f"{topic.decode('utf-8')}: {message.decode('utf-8')}")
                except zmq.Again:
                    print("No message received within our timeout period")
                except KeyboardInterrupt:
                    self.connected = False
            else:
                print("Subscriber thread interrupted, cleaning up...")
                sub_socket.close()
                sub_context.term()
                break

    def publisher_thread(self):
        pub_context, pub_socket = self.initialize_publisher()
        while True:
            if self.connected:
                for topic in self.pub_topic:
                    # message = self.process_message(f'{topic}', time.time())
                    # message = self.process_message(topic, self.pub_messages[topic])
                    message = f"{self.pub_messages[topic]}"
                    pub_socket.send_multipart([topic, message.encode("utf-8")])
            else:
                print("Publisher thread interrupted, cleaning up...")
                pub_socket.close()
                pub_context.term()
                break

    def run(self):
        self.connected = True
        pub_thread = threading.Thread(target=self.publisher_thread)
        sub_thread = threading.Thread(target=self.subscriber_thread)
        sub_thread.start()
        pub_thread.start()

        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.connected = False
            time.sleep(0.1)
            print("Main thread interrupted, cleaning up...")
            sub_thread.join()
            pub_thread.join()


# if __name__ == "__main__":
#     global posegraph = PoseGraph()

#     hl1_manager = ZMQManager("")
#     hl1_manager.run()
#     hl1_manager.msg
