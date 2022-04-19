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
#         tag_dict = {
#             "8":{"intersection":"A4","coords":(1.74,2.93),"orient":math.pi/2},
#             "13":{"intersection":"B3","coords":(1.47,1.82),"orient":-math.pi/2},
# #             "14":{"intersection":"D3","coords":(1.22,0.635),"orient":math.pi},
#             "15":{"intersection":"E4","coords":(2.535,0.045),"orient":-math.pi/2},
# #             "21":{"intersection":"C4","coords":(1.815,1.48),"orient":math.pi},
# #             "22":{"intersection":"E4","coords":(2.08,0.56),"orient":math.pi/2},
#             "23":{"intersection":"C5","coords":(2.925,1.82),"orient":0},
#             "24":{"intersection":"A3","coords":(1.12,2.93),"orient":math.pi/2},
#             "35":{"intersection":"D3","coords":(1.74,1.155),"orient":0},
#             "36":{"intersection":"B1","coords":(0.56,2.335),"orient":0},
# #             "37":{"intersection":"E3","coords":(1.22,0.56),"orient":math.pi/2},
#             "38":{"intersection":"C1","coords":(0.045,1.75),"orient":math.pi/2},
#             "44":{"intersection":"B4","coords":(2.335,2.335),"orient":0},
#             "45":{"intersection":"B4","coords":(2.335,1.82),"orient":-math.pi/2},
#             "46":{"intersection":"B4","coords":(1.815,2.335),"orient":math.pi/2},
#             "47":{"intersection":"B4","coords":(1.815,1.82),"orient":math.pi},
#             "59":{"intersection":"A1","coords":(0.425,2.93),"orient":math.pi/2},
# #             "63":{"intersection":"E2","coords":(0.56,0.56),"orient":math.pi/2},
# #             "67":{"intersection":"C2","coords":(0.88,1.225),"orient":-math.pi/2},
#             "75":{"intersection":"B2","coords":(0.87,1.82),"orient":-math.pi/2},
#             "76":{"intersection":"D1","coords":(0.045,0.605),"orient":math.pi},
#             "77":{"intersection":"C5","coords":(2.55,1.505),"orient":math.pi},
# #             "78":{"intersection":"C3","coords":(1.115,1.75),"orient":math.pi/2},
# #             "89":{"intersection":"D4","coords":(1.815,1.115),"orient":math.pi/2},
# #             "90":{"intersection":"D5","coords":(2.925,1.155),"orient":0},
#             "91":{"intersection":"D4","coords":(2.335,0.635),"orient":-math.pi/2},
# #             "92":{"intersection":"D4","coords":(1.815,0.635),"orient":math.pi},
#             "164":{"intersection":"A2","coords":(0.965,2.41),"orient":-math.pi/2},
# #             "166":{"intersection":"D5","coords":(2.925,0.635),"orient":-math.pi/2},
#             "190":{"intersection":"A3","coords":(1.59,2.41),"orient":-math.pi/2},
# #             "191":{"intersection":"E1","coords":(0.635,0.045),"orient":-math.pi/2},
#             "192":{"intersection":"B1","coords":(0.045,2.335),"orient":math.pi/2},
# #             "205":{"intersection":"E3","coords":(1.22,0.045),"orient":math.pi},
# #             "206":{"intersection":"E3","coords":(1.74,0.045),"orient":-math.pi/2},
# #             "207":{"intersection":"D3","coords":(1.22,1.155),"orient":math.pi/2},
#             "226":{"intersection":"B1","coords":(0.045,1.82),"orient":math.pi},
# #             "227":{"intersection":"C1","coords":(0.045,1.225),"orient":math.pi},
# #             "228":{"intersection":"C1","coords":(0.56,1.75),"orient":0},
# #             "303":{"intersection":"D5","coords":(2.415,0.635),"orient":math.pi},
#             "304":{"intersection":"B3","coords":(1.155,2.335),"orient":math.pi/2},
#         }
        # following dictionary takes into account offset to mid point between clear tape and tag
        tag_dict = {
            "8":{"intersection":"B4","coords":(1.89,2.83),"orient":0},
            "13":{"intersection":"B1","coords":(1.32,1.87),"orient":-math.pi/2},
            "14":{"intersection":"D3","coords":(1.27,0.785),"orient":math.pi},
            "15":{"intersection":"E4","coords":(2.385,0.095),"orient":-math.pi/2},
            "21":{"intersection":"B4","coords":(1.815,1.63),"orient":math.pi},
            "22":{"intersection":"E4","coords":(1.93,0.51),"orient":math.pi/2},
            "23":{"intersection":"D5","coords":(2.925,1.67),"orient":0},
            "24":{"intersection":"B4","coords":(1.27,2.88),"orient":0},
            "35":{"intersection":"D3","coords":(1.74,1.005),"orient":0},
            "36":{"intersection":"B1","coords":(0.56,2.185),"orient":0},
            "37":{"intersection":"E3","coords":(1.37,0.56),"orient":math.pi/2},
            "38":{"intersection":"C1","coords":(0.195,1.75),"orient":math.pi/2},
            "44":{"intersection":"B4","coords":(2.335,2.185),"orient":0},
            "45":{"intersection":"B4","coords":(2.185,1.82),"orient":-math.pi/2},
            "46":{"intersection":"B4","coords":(1.965,2.335),"orient":math.pi/2},
            "47":{"intersection":"B4","coords":(1.815,1.97),"orient":math.pi},
            "59":{"intersection":"B4","coords":(0.575,2.88),"orient":0},
            "63":{"intersection":"E2","coords":(0.41,0.51),"orient":math.pi/2},
            "67":{"intersection":"C1","coords":(0.73,1.275),"orient":-math.pi/2},
            "75":{"intersection":"B1","coords":(0.72,1.87),"orient":-math.pi/2},
            "76":{"intersection":"D1","coords":(0.095,0.755),"orient":math.pi},
            "77":{"intersection":"B4","coords":(2.55,1.655),"orient":-math.pi/2},
            "78":{"intersection":"D3","coords":(1.265,1.65),"orient":0},
            "89":{"intersection":"D4","coords":(1.965,1.115),"orient":math.pi/2},
            "90":{"intersection":"D5","coords":(2.925,1.005),"orient":0},
            "91":{"intersection":"D4","coords":(2.185,0.685),"orient":-math.pi/2},
            "92":{"intersection":"D4","coords":(1.815,0.785),"orient":math.pi},
            "164":{"intersection":"A2","coords":(0.815,2.46),"orient": 0},
            "166":{"intersection":"D5","coords":(2.775,0.635),"orient":-math.pi/2},
            "190":{"intersection":"B1","coords":(1.44,2.46),"orient":0},
            "191":{"intersection":"E1","coords":(0.485,0.055),"orient":-math.pi/2},
            "192":{"intersection":"B1","coords":(0.195,2.335),"orient":math.pi/2},
            "205":{"intersection":"E3","coords":(1.22,0.195),"orient":math.pi},
            "206":{"intersection":"E3","coords":(1.59,0.095),"orient":-math.pi/2},
            "207":{"intersection":"D3","coords":(1.37,1.155),"orient":math.pi/2},
            "226":{"intersection":"B1","coords":(0.095,1.97),"orient":math.pi},
            "227":{"intersection":"C1","coords":(0.095,1.375),"orient":math.pi},
            "228":{"intersection":"C1","coords":(0.56,1.6),"orient":0},
            "303":{"intersection":"D5","coords":(2.415,0.785),"orient":math.pi},
            "304":{"intersection":"B4","coords":(1.305,2.285),"orient":math.pi/2},
        }
        return tag_dict
