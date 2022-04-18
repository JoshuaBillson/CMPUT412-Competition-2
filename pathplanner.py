#!/usr/bin/env python3
import csv

class Planner():
    def __init__(self):
        tag_order = []

    def get_tag(self):
        order_path = "/code/catkin_ws/src/CMPUT412-Competition-2/packages/competition2/apriltag_order_competition2.csv"
        with open(order_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                tag_order = row
        
        return tag_order