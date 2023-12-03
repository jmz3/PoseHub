import zmq
import time
import sys
import threading

class ZMQManager:
    def __init__(self, sub_ip, sub_port, pub_port, sub_topic, pub_topic):
        self.sub_ip = sub_ip
        self.sub_port = sub_port
        self.pub_port = pub_port
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.connected = False

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
    def process_message(message):
        return f"Processed {message}"

    def subscriber_thread(self):
        sub_context, sub_socket = self.initialize_subscriber()
        while True:
            if self.connected:
                try:
                    topic, message = sub_socket.recv_multipart()
                    print(f"Received: {message.decode('utf-8')}")
                except zmq.Again:
                    print('No message received within our timeout period')
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
                processed_message = self.process_message(time.time())
                pub_socket.send_multipart([self.pub_topic, processed_message.encode('utf-8')])

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