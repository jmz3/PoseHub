import json
from typing import Dict, Any
import numpy as np

class DataRecorder:
    def __init__(self, filename: str):
        """
        Initializes the DataRecorder class.
        :param filename: Path to the JSON file where data will be stored.
        """
        self.filename = filename
        self.data = {"scene_objects": {}}
        self.nodes = []
        self.edges = dict([]) #Initialise both as empty

    def update(self,nodes: Any, edges: Any):
        self.nodes = nodes
        self.edges = edges

    def snapshot(self,get_transform,duration): #Pass the timestamp directly to the save function
        """
        Copies the graph condition at each time-stamp i.e makes a snapshot and puts it into the file
        """
        #self.data["scene_objects"] = {}
        duration_since_start = duration
        #For each node, save the transformation, flag and timestamp
        #print("Node format example:", self.nodes[0])
        for [prop,node_id] in self.nodes:
            connections = self.edges.get(node_id,{})
            for child_id, (T,active) in connections.items():
                if T is None:
                    #If transform does not exist, get by tracking method
                    transform = get_transform(node_id,child_id,solver_method='BFS') #Randomnly set BFS
                    flag = False #Since we are tracking by calculated method
                    break

                #if transform exists, save everything
                transform = T.tolist()
                flag = active

            self.data["scene_objects"][duration_since_start] = {
                "node": node_id,
                "type": prop,
                "transformation": transform,
                "flag": flag
            } 

    def save(self):
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
            print(f"File {self.filename} not found")
    
    def clear(self):
        """
        Clears the recorded data.
        """
        self.data = {"scene_objects": {}}