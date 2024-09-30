import zmq
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot

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

    artool = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0]) # translation, rotation, track_flag

    # Ensure the socket is ready
    time.sleep(1)
    try:
        while True:
            # generate a random velocity for pose_artool
            rand_vel_trans = np.random.uniform(-0.001, 0.001, 3)
            rand_vel_rot = np.random.uniform(-0.001, 0.001, 3)
            # artool = artool + np.concatenate(rand_vel_trans, rand_vel_rot)
            curr_rot = Rot.from_quat(artool[3:7]).as_euler("zxy") + rand_vel_rot
            artool[0:3] += rand_vel_trans
            artool[3:7] = Rot.from_euler("zxy", curr_rot).as_quat()

            pub_messages["artool"] = ",".join([str(i) for i in artool]) + ',1'

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