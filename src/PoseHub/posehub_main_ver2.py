from typing import Type, Dict, List, Optional
from comm.ZMQManager import ZMQManager
from pose_graph import PoseGraph
import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
from visualize.viz_frames import axis_init
from visualize.viz_frames import generate_frames
import open3d as o3d


def main(args):
    """
    Main function
    """
    global memoryBuffer
    memoryBuffer = []
    # initialize the pose graph
    pose_graph = PoseGraph()

    # # initialize the visualization code snippet
    # figure = plt.figure()
    # plt.ion()
    # plt.show()
    # ax = figure.add_subplot(projection="3d")
    # ax = axis_init(ax, 1.5, "Pose")

    tool_1_id = args.sub_topic[0]
    tool_2_id = args.sub_topic[1]
    tool_3_id = args.sub_topic[2]

    sensor_1_id = "h1"
    # sensor_2_id = "h2"

    # frames, frame_pm = generate_frames(
    #     ax=ax,
    #     sensors=[sensor_1_id, sensor_2_id],
    #     # sensors=[sensor_2_id],
    #     objects=[tool_1_id, tool_2_id, tool_3_id],
    #     axis_length=0.5,
    # )  # generate the frames for visualization,
    # the number of frames is equal to the number of sensors * objects
    # frames is a dictionary with the following structure:
    # frames = {
    #     "sensor_1": {
    #         "tool_1": {
    #             "x": None,
    #             "y": None,
    #             "z": None,
    #             "sensor_tag": None,
    #             "frame_id": None,
    #         },
    #         "tool_2": {
    #             ...
    #         },
    #        ...
    #     },
    #    ...
    # }
    # frame_pm is a (4x4) ndarray, containing 1 origin point and 3 axes end-point of the frames

    args_1 = argparse.Namespace(
        sub_ip=args.sub_ip_1,
        sub_port="5588",
        pub_port="5589",
        sub_topic=[tool_1_id, tool_2_id, tool_3_id],
        pub_topic=[tool_1_id, tool_2_id, tool_3_id],
        sensor_name=sensor_1_id,
    )  # args for h1 sensor

    # args_2 = argparse.Namespace(
    #     sub_ip=args.sub_ip_2,
    #     sub_port="5581",
    #     pub_port="5580",
    #     sub_topic=[tool_1_id, tool_2_id, tool_3_id],
    #     pub_topic=[tool_1_id, tool_2_id, tool_3_id],
    #     sensor_name=sensor_2_id,
    # )  # args for h2 sensor

    zmq_manager_1 = ZMQManager(
        sub_ip=args_1.sub_ip,
        sub_port=args_1.sub_port,
        pub_port=args_1.pub_port,
        sub_topic=args_1.sub_topic,
        pub_topic=args_1.pub_topic,
        sensor_name=args_1.sensor_name,
    )

    # zmq_manager_2 = ZMQManager(
    #     sub_ip=args_2.sub_ip,
    #     sub_port=args_2.sub_port,
    #     pub_port=args_2.pub_port,
    #     sub_topic=args_2.sub_topic,
    #     pub_topic=args_2.pub_topic,
    #     sensor_name=args_2.sensor_name,
    # )

    zmq_manager_1.initialize()
    # zmq_manager_2.initialize()

    # pose_graph.add_sensor("h1")
    # pose_graph.add_sensor("h2")

    # create the visualization of the axes in the 3D space
    o3dviz = o3d.visualization.Visualizer()
    o3dviz.create_window()

    frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[1, 1, 0]
    )
    frame_2 = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0]
    )
    frame_3 = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[1, -1, 0]
    )
    o3dviz.add_geometry(frame_1)
    o3dviz.add_geometry(frame_2)
    o3dviz.add_geometry(frame_3)

    camera_handle = o3dviz.get_view_control()
    camera_handle.set_lookat([0, 0, 0])
    camera_handle.set_front([0, 0, 1])
    camera_handle.set_up([1, -1, 0])
    camera_handle.set_zoom(1.5)
    view_params = camera_handle.convert_to_pinhole_camera_parameters()

    o3dviz.poll_events()
    o3dviz.update_renderer()

    try:
        while True:
            # Running the main loop
            start_time = time.time()
            # receive messages
            poseinfo_sensor1 = zmq_manager_1.receive_poses()
            # poseinfo_sensor2 = zmq_manager_2.receive_poses()
            camera_handle = o3dviz.get_view_control()
            camera_handle.set_lookat([0, 0, 0])
            camera_handle.set_front([0, 0, 1])
            camera_handle.set_up([1, -1, 0])
            camera_handle.set_zoom(1.5)
            o3dviz.poll_events()
            o3dviz.update_renderer()

            if len(poseinfo_sensor1) != 0:
                pose_graph.update_graph("h1", poseinfo_sensor1)
            else:
                print("poseinfo_sensor1 is empty")

            # if len(poseinfo_sensor2) != 0:
            #     pose_graph.update_graph("h2", poseinfo_sensor2)
            #     # print("poseinfo: ", poseinfo_sensor2)
            # else:
            #     print("poseinfo_sensor2 is empty")

            print("Time for pose graph update: ", time.time() - start_time)
            # # send messages

            for topic in args_1.pub_topic:
                # transfer the topic from bytes to string

                pose = pose_graph.get_transform("h1", topic, solver_method="BFS")
                if pose is not None and np.linalg.norm(pose[:3, 3]) > 0.00001:
                    print("pose for frame 1: ", pose)
                    zmq_manager_1.send_poses(topic, pose)

                    # update the visualization of the axes in the 3D space
                    if topic == tool_1_id:
                        # o3dviz.remove_geometry(frame_1)
                        # frame_1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=pose[:3, 3])
                        # frame_1.transform(pose)
                        # o3dviz.add_geometry(frame_1)
                        position_increment = pose[:3, 3] - frame_1.get_center()
                        frame_1.translate(position_increment)
                        o3dviz.update_geometry(frame_1)
                        camera_handle = o3dviz.get_view_control()
                        camera_handle.set_lookat([0, 0, 0])
                        camera_handle.set_front([0, 0, 1])
                        camera_handle.set_up([1, -1, 0])
                        camera_handle.set_zoom(1.5)
                        o3dviz.poll_events()
                        o3dviz.update_renderer()

            # for topic in args_2.pub_topic:
            #     # transfer the topic from bytes to string
            #     pose = pose_graph.get_transform("h2", topic, solver_method="BFS")
            #     if pose is not None and np.linalg.norm(pose[:3, 3]) > 0.00001:
            #         zmq_manager_2.send_poses(topic, pose)

            print("edges after graph search: ", pose_graph.edges)

    except KeyboardInterrupt:
        zmq_manager_1.terminate()
        # zmq_manager_2.terminate()
        # plt.ioff()
        o3dviz.destroy_window()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Python script for running the AR tool tracking",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sub_ip_1",
        default="10.203.70.145",
        type=str,
        help="subscriber ip address sensor 1",
    )
    # parser.add_argument(
    #     "--sub_ip_2",
    #     default="10.203.232.86",
    #     type=str,
    #     help="subscriber ip address sensor 2",
    # )
    parser.add_argument(
        "--sub_topic",
        default=["artool", "ref_1", "phantom"],
        type=str,
        help="subscriber topics",
    )
    parser.add_argument(
        "--pub_topic",
        default=["artool", "ref_1", "phantom"],
        type=str,
        help="publisher topic",
    )
    args = parser.parse_args()

    main(args)
