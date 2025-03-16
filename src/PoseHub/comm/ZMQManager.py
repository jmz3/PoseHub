import zmq
import time
import sys
import threading
import numpy as np
from scipy.spatial.transform import Rotation as Rot


class ZMQManager:
    def __init__(self, sub_ip, sub_port, pub_port, sub_topic, pub_topic, sensor_name):
        self.sub_ip = sub_ip
        self.sub_port = sub_port
        self.pub_port = pub_port
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.sensor_name = sensor_name
        self.connected = False
        self.pub_thread = None
        self.sub_thread = None
        self.pub_messages = {
            topic: f"{topic} Waiting for pub message..." for topic in self.pub_topic
        }
        self.sub_poses = {
            topic: f"{topic} Waiting for sub message..." for topic in self.sub_topic
        }
        # self.sub_poses = defaultdict()

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
            subscriber.setsockopt(zmq.SUBSCRIBE, t.encode("utf-8"))

        return context, subscriber

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
                except zmq.Again:
                    print(
                        f"{self.sensor_name}: No message received within our timeout period"
                    )
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
                    message = f"{self.pub_messages[topic]}"
                    pub_socket.send_multipart(
                        [topic.encode("utf-8"), message.encode("utf-8")]
                    )
            else:
                print("Publisher thread interrupted, cleaning up...")
                pub_socket.close()
                pub_context.term()
                break

    def initialize(self):
        """
        initialize a ZMQManager
        """
        self.connected = True
        self.pub_thread = threading.Thread(target=self.publisher_thread)
        self.sub_thread = threading.Thread(target=self.subscriber_thread)
        self.sub_thread.start()
        self.pub_thread.start()

    def terminate(self):
        """
        terminate a ZMQManager
        """
        self.connected = False
        time.sleep(0.1)
        print("Main thread interrupted, cleaning up...")
        self.sub_thread.join()
        self.pub_thread.join()

    def receive_poses(self):
        """
        Receive poses from the sensors

        Args:
        ----------
            args: argparse.Namespace, arguments
            zmq_manager: ZMQManager, the ZMQManager object

        Return:
        ----------
            tool_info: dict, {topic name: [transformation matrix: np.ndarray(np.float32, (4, 4)), isActive: bool]}
        """
        received_dict = self.sub_poses
        pose_mtx = np.identity(4)
        tool_info = {}
        for topic in self.sub_topic:
            if topic in received_dict:
                if len(received_dict[topic].split(",")) < 7:
                    tool_info[topic] = [np.identity(4), False]
                    continue
                else:
                    twist = np.array(
                        [float(x) for x in received_dict[topic].split(",")]
                    )
                    pose_mtx = np.identity(4)
                    quat_left = twist[3:7]
                    pose_mtx[:3, :3] = Rot.from_quat(quat_left).as_matrix()
                    pose_mtx[:3, 3] = np.array([twist[0], twist[1], twist[2]])

                    tool_info[topic] = [
                        pose_mtx,
                        twist[7] == 1,
                    ]
            else:
                print("Subscribing to the topic: ", topic, " but no message received")
                return {}
        return tool_info

    def send_poses(self, topic, transform_mtx: np.ndarray):
        """
        Send poses to the sensors
        """

        # decode the matrix into twist
        # check the size of the matrix
        if transform_mtx.shape != (4, 4):
            print("The size of the transformation matrix is not 4x4")
            return

        # # convert to unity convention
        quat = Rot.from_matrix(transform_mtx[:3, :3]).as_quat().reshape(1, -1)
        trans = transform_mtx[:3, 3].reshape(1, -1)

        new_pose = np.hstack([trans, quat])
        # Convert to string
        new_pose_str = ",".join(str(num) for num in new_pose.flatten())
        self.pub_messages[topic] = new_pose_str + ",1"
