#!/usr/bin/env python3

import math
import csv
from tag import Tag

class Planner():
    def __init__(self):
        tag_order = []

    def get_tag(self):
        order_path = "/code/catkin_ws/src/lab_4/packages/lab_4/apriltag_order_competition2.csv"
        with open(order_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                tag_order = int(row)
        
        return tag_order

    def get_dict(self):
        tag_dict = {
            "8":{"tile":"A4","coords":(1.74,2.93),"orient":math.pi/2},
            "13":{"tile":"B3","coords":(1.47,1.82),"orient":-math.pi/2},
            "14":{"tile":"D3","coords":(1.22,0.635),"orient":math.pi},
            "15":{"tile":"E4","coords":(2.535,0.045),"orient":-math.pi/2},
            "21":{"tile":"C4","coords":(1.815,1.48),"orient":math.pi},
            "22":{"tile":"E4","coords":(2.08,0.56),"orient":math.pi/2},
            "23":{"tile":"C5","coords":(2.925,1.82),"orient":0},
            "24":{"tile":"A3","coords":(1.12,2.93),"orient":math.pi/2},
            "35":{"tile":"D3","coords":(1.74,1.155),"orient":0},
            "36":{"tile":"B1","coords":(0.56,2.335),"orient":0},
            "37":{"tile":"E3","coords":(1.22,0.56),"orient":math.pi/2},
            "38":{"tile":"C1","coords":(0.045,1.75),"orient":math.pi/2},
            "44":{"tile":"B4","coords":(2.335,2.335),"orient":0},
            "45":{"tile":"B4","coords":(2.335,1.82),"orient":-math.pi/2},
            "46":{"tile":"B4","coords":(1.815,2.335),"orient":math.pi/2},
            "47":{"tile":"B4","coords":(1.815,1.82),"orient":math.pi},
            "59":{"tile":"A1","coords":(0.425,2.93),"orient":math.pi/2},
            "63":{"tile":"E2","coords":(0.56,0.56),"orient":math.pi/2},
            "67":{"tile":"C2","coords":(0.88,1.225),"orient":-math.pi/2},
            "75":{"tile":"B2","coords":(0.87,1.82),"orient":-math.pi/2},
            "76":{"tile":"D1","coords":(0.045,0.605),"orient":math.pi},
            "77":{"tile":"C5","coords":(2.55,1.505),"orient":math.pi},
            "78":{"tile":"C3","coords":(1.115,1.75),"orient":math.pi/2},
            "89":{"tile":"D4","coords":(1.815,1.115),"orient":math.pi/2},
            "90":{"tile":"D5","coords":(2.925,1.155),"orient":0},
            "91":{"tile":"D4","coords":(2.335,0.635),"orient":-math.pi/2},
            "92":{"tile":"D4","coords":(1.815,0.635),"orient":math.pi},
            "164":{"tile":"A2","coords":(0.965,2.41),"orient":-math.pi/2},
            "166":{"tile":"D5","coords":(2.925,0.635),"orient":-math.pi/2},
            "190":{"tile":"A3","coords":(1.59,2.41),"orient":-math.pi/2},
            "191":{"tile":"E1","coords":(0.635,0.045),"orient":-math.pi/2},
            "192":{"tile":"B1","coords":(0.045,2.335),"orient":math.pi/2},
            "205":{"tile":"E3","coords":(1.22,0.045),"orient":math.pi},
            "206":{"tile":"E3","coords":(1.74,0.045),"orient":-math.pi/2},
            "207":{"tile":"D3","coords":(1.22,1.155),"orient":math.pi/2},
            "226":{"tile":"B1","coords":(0.045,1.82),"orient":math.pi},
            "227":{"tile":"C1","coords":(0.045,1.225),"orient":math.pi},
            "228":{"tile":"C1","coords":(0.56,1.75),"orient":0},
            "303":{"tile":"D5","coords":(2.415,0.635),"orient":math.pi},
            "304":{"tile":"B3","coords":(1.155,2.335),"orient":math.pi/2},
        }

        return tag_dict