#!/usr/bin/env python3

class MapGraph:
    def __init__(self):
        self.location = 0
        self.destination = 0
        self.nodes = {0: Node(0), 1: Node(1)}

    def get_coords(self, tag_id):
        return 0, 0
    
    def set_plan(self, location, destination):
        return self.nodes[destination]
    

class Node:
    def __init__(self, tag_id):
        self.id = 0
        self.neighbors = []
    
    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)
