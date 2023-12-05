# PoseHub
PoseHub is a library that manages the transformations of different sensors and objects in the same scene and provides a unified interface for accessing them.

## Data Structure
Currently, the basic data structure is a customized graph class called `PoseGraph`. It is a directed graph that stores the transformation between nodes. The transformation is represented by a `Pose` class, which contains a 3x3 rotation matrix and a 3x1 translation vector. The `PoseGraph` class provides a unified interface for accessing the transformation between any two nodes. The transformation between two nodes is the product of all the transformations along the path between them. The `PoseGraph` class also provides a unified interface for accessing the transformation between any node and the root node. The transformation between a node and the root node is the product of all the transformations along the path from the node to the root node.

Potential future work: borrow the idea of [networkx](https://networkx.org/) or directly use networkx to implement the graph class if it is more efficient.

## Threading and Locking
The communication objects are running in different threads. They are created and started in the main thread. The main thread is responsible for creating and starting the communication objects. The communication objects are responsible for receiving data from the sensors and updating the `PoseGraph` object. The `PoseGraph` object is declared in the main thread so that it can be accessed by all the communication objects. The `PoseGraph` object is locked when it is being updated by the communication objects. The `PoseGraph` object is unlocked when it is being accessed by the main thread.