#!/usr/bin/env python3

class Map:
    def __init__(self):
        self.paths = []
        self.visited = []
        self.queue = []
        self.tiles = {
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

    def get_path(self, origin, destination):
        self.visited.append(origin)
        self.queue.append(origin)
        self.paths = [[origin]]
        while len(self.paths) > 0:
            path = self.paths.pop(0)
            if self.arrived(path, destination):
                return path
            else:    
                neighbors = self.tiles[path[-1]]
                for neighbor in neighbors:
                    self.paths.append(path + [neighbor])

    @staticmethod
    def arrived(path, destination):
        return path[-1] == destination
        

if __name__ == '__main__':
    map = Map()
    print(map.get_path("A3", "E5"))
    