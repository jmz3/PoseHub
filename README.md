# PoseHub
PoseHub is a library that manages the transformations of different sensors and objects in the same scene and provides a unified interface for accessing them. For the purpose of simplification, this package only depends on the [numpy](https://numpy.org/) and [matplotlib]() package. The package is designed to be used in the [AR Project, Surgical Tool Tracking using Multi-Sensor System]()

## Installation

```bash
git clone https://github.com/jmz3/PoseHub.git
```

## Usage
Open a terminal and run the following command to start the ZMQ server.
Note that you need to modify the IP address in the `posehub_main.py` file to the IP address of your sensor (either HoloLens or something else).
```bash
```bash
python3 posehub_main.py
```

If you have multiple sensors, you can copy the following code snippet and modify the args to create multiple communication objects.
```python
args_1 = argparse.Namespace(
    sub_ip=args.sub_ip_1,
    sub_port="5588",
    pub_port="5589",
    sub_topic=[tool_1_id, tool_2_id, tool_3_id],
    pub_topic=[tool_1_id, tool_2_id, tool_3_id],
    sensor_name=sensor_1_id,
)  # args for h1 sensor

# create zmq manager
zmq_manager_1 = ZMQManager(
    sub_ip=args_1.sub_ip,
    sub_port=args_1.sub_port,
    pub_port=args_1.pub_port,
    sub_topic=args_1.sub_topic,
    pub_topic=args_1.pub_topic,
    sensor_name=args_1.sensor_name,
)

# initialize zmq manager
zmq_manager_1.initialize() 

# create pose graph object to store the transformation
pose_graph = PoseGraph()
pose_graph.add_sensor("h1")
```

## Data Structure
Currently, the basic data structure is a customized graph class called `PoseGraph`. It is a directed graph that stores the transformation between nodes. The transformation is represented by a `Pose` class, which contains a 3x3 rotation matrix and a 3x1 translation vector. The `PoseGraph` class provides a unified interface for accessing the transformation between any two nodes. The transformation between two nodes is the product of all the transformations along the path between them. The `PoseGraph` class also provides a unified interface for accessing the transformation between any node and the root node. The transformation between a node and the root node is the product of all the transformations along the path from the node to the root node.

Potential future work: borrow the idea of [networkx](https://networkx.org/) or directly use networkx to implement the graph class if it is more efficient.

## Threading and Locking
The communication objects are running in different threads. They are created and started in the main thread. The main thread is responsible for creating and starting the communication objects. The communication objects are responsible for receiving data from the sensors and updating the `PoseGraph` object. The `PoseGraph` object is declared in the main thread so that it can be accessed by all the communication objects. The `PoseGraph` object is locked when it is being updated by the communication objects. The `PoseGraph` object is unlocked when it is being accessed by the main thread.

## Contributors
* [Jiaming Zhang](https://github.com/jmz3)
* [Hongchao Shu](https://github.com/Soooooda69)