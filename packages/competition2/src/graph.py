#!/usr/bin/env python3

class Map:
    def __init__(self):
        self.tiles = {
            "A1": Tile("A1", ["A2", "B1"]), 
            "A2": Tile("A2", ["A1", "A3"]), 
            "A3": Tile("A3", ["A2", "A4"]), 
            "A4": Tile("A4", ["A3", "B4"]), 
            "B1": Tile("B1", ["A2", "B1"]), 
            "B2": Tile("B2", ["A2", "B1"]), 
            "B3": Tile("B3", ["A2", "B1"]), 
            "B4": Tile("B4", ["A2", "B1"]), 
            "B5": Tile("B5", ["A2", "B1"]), 
            "C1": Tile("C1", ["A2", "B1"]), 
            "C2": Tile("C2", ["A2", "B1"]), 
            "C3": Tile("C3", ["A2", "B1"]), 
            "C4": Tile("C4", ["A2", "B1"]), 
            "C5": Tile("C5", ["A2", "B1"]), 
            "D1": Tile("D1", ["A2", "B1"]), 
            "D2": Tile("D2", ["A2", "B1"]), 
            "D3": Tile("D3", ["A2", "B1"]), 
            "D4": Tile("D4", ["A2", "B1"]), 
            "D5": Tile("D5", ["A2", "B1"]), 
            "E1": Tile("E1", ["A2", "B1"]), 
            "E2": Tile("E2", ["A2", "B1"]), 
            "E3": Tile("E3", ["A2", "B1"]), 
            "E4": Tile("E4", ["A2", "B1"]), 
            "E5": Tile("E5", ["A2", "B1"]), 
            }

    def get_coords(self, tag_id):
        return 0, 0
    
    def set_plan(self, location, destination):
        return self.nodes[destination]
    

class Tile:
    def __init__(self, tag_id, neighbors, intersection=False):
        self.id = 0
        self.neighbors = neighbors
        self.intersection = intersection
