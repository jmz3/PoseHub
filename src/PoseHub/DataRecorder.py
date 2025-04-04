import json
from typing import Dict, Any
import numpy as np
from pose_graph import PoseGraph

class DataRecorder:
    #Pass the entire graph into the DataRecorder object
    def __init__(self, filename: str, graph: PoseGraph):
        """
        Initializes the DataRecorder class.
        :param filename: Path to the JSON file where data will be stored.
        """
        self.filename = filename
        self.data = {"scene_objects": {}}
        self.graph = graph
    
    def save(self,duration):
        """
        Saves the recorded data to the JSON file.
        """
        self.data["scene_objects"] = {}
        duration_since_start = duration
        #For each node, save the transformation, flag and timestamp
        print("Node format example:", self.graph.nodes[0])
        for [prop,node_id] in self.graph.nodes:
            connections = self.graph.edges.get(node_id,{})
            for child_id, (T,active) in connections.items():
                if T is None:
                    #If transform does not exist, get by tracking method
                    transform = self.graph.get_transform(node_id,child_id,solver_method='BFS') #Randomnly set BFS
                    flag = False #Since we are tracking by calculated method
                    break

                #if transform exists, save everything
                transform = T.tolist()
                flag = active

            
            self.data["scene_objects"][node_id] = {
                "type": prop,
                "time_elapsed": duration_since_start,
                "transformation": transform,
                "flag": flag
            } 

        with open(self.filename, "w") as file:
            json.dump(self.data, file, indent=4)
    











    def load(self):
        """
        Loads existing data from the JSON file.
        """
        try:
            with open(self.filename, "r") as file:
                self.data = json.load(file)
        except FileNotFoundError:
            print(f"File {self.filename} not found. Starting fresh.")
    
    def clear(self):
        """
        Clears the recorded data.
        """
        self.data = {"nodes": {}, "edges": {}}