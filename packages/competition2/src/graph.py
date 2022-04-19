#!/usr/bin/env python3
import math

class Map:
    def __init__(self):
        self.paths = []
        self.visited = []
        self.queue = []
        self.foo = {
            "A1": ["A2", "B1"], 
            "A2": ["A1", "A3"], 
            "A3": ["A2", "A4"], 
            "A4": ["A3", "B4"], 
            "B1": ["A1", "B2", "C1"], 
            "B2": ["B1", "B3"], 
            "B3": ["B2", "B4"], 
            "B4": ["B3", "B5", "A4", "C4"], 
            "B5": ["B4", "C5"], 
            "C1": ["B1", "C2", "D1"], 
            "C2": ["C1", "C3"], 
            "C3": ["C2", "D3"], 
            "C4": ["B4", "D4"], 
            "C5": ["B5", "D5"], 
            "D1": ["C1", "E1"], 
            "D3": ["C3", "D4", "E3"], 
            "D4": ["D3", "C4", "D5"], 
            "D5": ["C5", "D4", "E5"],
            "E1": ["D1", "E2"], 
            "E2": ["E1", "E3"], 
            "E3": ["E2", "C3", "E4"],
            "E4": ["E3", "E5"], 
            "E5": ["E4", "D5"], 
        }
        self.tiles = {
            "B1": [("C1", math.pi), ("B4", -math.pi / 2)], 
            "B4": [("B1", math.pi / 2), ("D4", math.pi), ("D5", -math.pi / 2)], 
            "C1": [("B1", 0), ("D3", -math.pi / 2), ("E3", math.pi)], 
            "D3": [("C1", 0), ("D4", -math.pi/2), ("E3", math.pi)], 
            "D4": [("D3", math.pi / 2), ("B4", 0), ("D5", -math.pi / 2)], 
            "D5": [("D4", math.pi / 2), ("B4", 0), ("E3", math.pi)], 
            "E3": [("C1", math.pi / 2), ("D3", 0), ("D5", -math.pi / 2)],
        }

    def get_path(self, origin, destination):
        self.visited.append(origin)
        self.queue.append(origin)
        self.paths = [[origin]]
        counter = 0
        while True:
            counter += 1
            path = self.paths.pop(0)
            if self.arrived(path, destination):
                path_head = path.pop(-1)
                path.append((path_head, destination[1]))
                return path
            else:    
                neighbors = self.tiles[path[-1]]
                for neighbor in neighbors:
                    path_head = path[-1]
                    self.paths.append(path[:-1] + [(path_head, neighbor[1])] + [neighbor[0]])

    @staticmethod
    def arrived(path, destination):
        #print(f"Path: {path[-1]}, destination: {destination[0]}")
        return path[-1] == destination[0]
        

if __name__ == '__main__':
    map = Map()
    print(map.get_path("D5", ("B1", 0)))
    
