import zmq
import time
import numpy as np

def main():
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    # publisher.bind(f"tcp://*:{self.pub_port}")
    publisher.bind("tcp://*:5588")

    pose_artool = ",".join([str(i) for i in np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])]) + ',1'
    pose_ref_1 = ",".join([str(i) for i in np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])]) + ',1'
    pose_phantom = ",".join([str(i) for i in np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])]) + ',1'

    pub_messages = {
        "artool": pose_artool,
        "ref_1": pose_ref_1,
        "phantom": pose_phantom,
    }

    # Ensure the socket is ready
    time.sleep(1)
    try:
        while True:
            for topic in ["artool", "ref_1", "phantom"]:
                message = f"{pub_messages[topic]}"
                publisher.send_multipart(
                    [topic.encode("utf-8"), message.encode("utf-8")]
                )
    except KeyboardInterrupt:
        print("Exiting...")
        publisher.close()
        context.term()      

if __name__ == "__main__":
    main()