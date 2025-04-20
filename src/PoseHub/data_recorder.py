import os
import json
from time import time
import numpy as np
from typing import Dict, Any


class DataRecorder:
    def __init__(self):
        """
        Initializes the DataRecorder class.
        :param filename: Path to the JSON file where data will be stored.
        """
        self.data = dict([])
        self.last_time = time()

    def update(self, edges: Any, edges_opt: Any):
        current_time = time()
        # We assume that both edges and edges_opt follow the definition:
        # {parent_id: {child_id: [transformation, isActive]}}
        if current_time - self.last_time > 0.2:
            # Only update if more than 0.2 seconds have passed since the last update.
            self.last_time = current_time
            # Store the edges in a serializable format.
            edges_ser = self.serialize_edges(edges)
            edges_opt_ser = self.serialize_edges(edges_opt)

            self.data[str(current_time)] = {
                "raw_graph": edges_ser,
                "opt_graph": edges_opt_ser,
            }

    def serialize_edges(self, edges: dict) -> dict:
        """
        Converts only the transformation part (4x4 NumPy array) in each edge to a list,
        while leaving the 'isActive' flag unchanged.
        Expected structure:
            edges = {parent_id: {child_id: [transformation, isActive]}, ...}
        """
        new_edges = {}
        for parent, children in edges.items():
            new_edges[parent] = {}
            for child, value in children.items():
                trans, isActive = value
                if isinstance(trans, np.ndarray):
                    trans_serial = trans.tolist()  # Convert 4x4 array to list
                else:
                    trans_serial = trans
                new_edges[parent][child] = [trans_serial, str(isActive)]
        return new_edges

    def save(self, filename: str):
        # Prevent accidentally overwriting an existing file.
        stamped_filename = filename + f"_{time.strftime('%Y%m%d_%H%M%S')}" + ".json"

        if os.path.exists(stamped_filename):
            raise FileExistsError(
                f"File {stamped_filename} already exists. Please choose a different filename."
            )

        # Process the stored data: assume that each record has both "raw_graph" and "opt_graph"
        with open(stamped_filename, "w") as file:
            json.dump(self.data, file, indent=4)
