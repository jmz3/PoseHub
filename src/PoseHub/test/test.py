import zmq
import time
import numpy as np
import scipy.spatial.transform.Rotation as Rot


def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5588")

    translation = np.array([1, 2, 3])
    rotation = Rot.Rotation.from_euler("xyz", [0, 0, 20], degrees=True).as_quat()
    print(rotation)
    pose = np.array([1, 2, 3, 4, 5, 6, 7])
    topic_1 = "artool"
    message_1 = ",".join([str(x) for x in pose])

    topic_2 = "reference"
    message_2 = ",".join([str(x) for x in pose])

    # Ensure the socket is ready
    time.sleep(1)

    while True:
        socket.send_multipart([topic_1.encode(), message_1.encode()])
        socket.send_multipart([topic_2.encode(), message_2.encode()])
        time.sleep(1)


if __name__ == "__main__":
    main()
