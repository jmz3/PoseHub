import matplotlib.pyplot as plt
import numpy as np
import zmq


class MessageSubscriber:
    def __init__(self, sub_ips, sub_ports, sub_topics):
        self.sub_ips = sub_ips
        self.sub_ports = sub_ports
        self.sub_topics = sub_topics

    def subscribe(self):
        # Subscribe to the ZMQ publisher
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(f"tcp://{self.sub_ips[0]}:{self.sub_ports[0]}")
        # Initialize the plot
        socket.setsockopt(zmq.RCVTIMEO, 1000)

        if isinstance(self.sub_topics[0], str):
            self.sub_topics[0] = self.sub_topics[0].encode()


# Usage example
if __name__ == "__main__":
    # Initialize the subscriber
    sub_ips = ["192.168.1.1", "192.168.1.0"]
    sub_ports = ["5588", "5589"]
    sub_topics = ["tool_1", "tool_2", "tool_3"]
    subscriber = MessageSubscriber(
        sub_ips=sub_ips, sub_ports=sub_ports, sub_topics=sub_topics
    )
    subscriber.subscribe()
