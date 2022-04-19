#!/usr/bin/env python3

import math
import numpy as np
import os
import rospy
from std_msgs.msg import Int32
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import smach
from competition2.msg import Localization
from trackers import LineTracker, TofTracker, LeftTracker
from threading import Lock, Thread
from graph import Map
from collections import deque

HOSTNAME = "/" + os.uname()[1]
MOTOR_TOPIC = HOSTNAME + "/car_cmd_switch_node/cmd"
LOCATION_TOPIC = HOSTNAME + "/output/location"
VELOCITY = 0.20
MAX_TOF_QUEUE = 5


class Fonda:
    def __init__(self):
        self.tag_order = []

    def get_intersection_from_tag(self, tag_id):
        # following dictionary takes into account offset to mid point between clear tape and tag
        tag_dict = {
            "8":{"intersection":"B4","coords":(1.89,2.83),"orient":0},
            "13":{"intersection":"B1","coords":(1.32,1.87),"orient":-math.pi/2},
            "14":{"intersection":"D3","coords":(1.27,0.785),"orient":math.pi},
            "15":{"intersection":"E3","coords":(2.385,0.095),"orient":-math.pi/2},
            "21":{"intersection":"B4","coords":(1.815,1.63),"orient":math.pi},
            "22":{"intersection":"D5","coords":(1.93,0.51),"orient":math.pi},
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
            "164":{"intersection":"B1","coords":(0.815,2.46),"orient": 0},
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
        return tag_dict[tag_id]

    def get_tag_at_orientation_and_intersection(self, intersection, angle):
        dictionary = {
            ("C1", 0): "228", 
            ("C1", -math.pi / 2): "67", 
            ("C1", math.pi / 2): "38", 
            ("C1", math.pi): "227", 
            ("B1", math.pi): "205", 
            ("B1", 0): "36", 
            ("B1", -math.pi / 2): "75", 
            ("B1", math.pi / 2): "192", 
            ("B1", math.pi): "226", 
            ("B4", 0): "44", 
            ("B4", -math.pi / 2): "46", 
            ("B4", math.pi / 2): "45", 
            ("B4", math.pi): "47", 
            ("D5", 0): "90", 
            ("D5", math.pi / 2): "89", 
            ("D5", math.pi): "303", 
            ("D4", 0): "44", 
            ("D4", -math.pi / 2): "91", 
            ("D4", math.pi / 2): "89", 
            ("D4", math.pi): "92", 
            ("D3", 0): "35", 
            ("D3", -math.pi / 2): "91", 
            ("D3", math.pi / 2): "207", 
            ("D3", math.pi): "14", 
            ("E3", 0): "35", 
            ("E3", -math.pi / 2): "206", 
            ("E3", math.pi / 2): "37", 
            ("E3", math.pi): "205", 
        }
        return dictionary[(intersection, angle)]

class MotorController:
    def __init__(self):
        self.pub = rospy.Publisher(MOTOR_TOPIC, Twist2DStamped, queue_size=1)
        self.msg = Twist2DStamped()

    def drive(self, angularVelocity, linearVelocity):
        self.msg.v = linearVelocity
        self.msg.omega = angularVelocity
        self.pub.publish(self.msg)

class LocalizationReader:
    def __init__(self):
        self.mutex = Lock()
        self.position = None
        self.orientation = None
        self.tags = []
        self.tile = None
        self.subscriber = rospy.Subscriber(LOCATION_TOPIC, Localization, self.callback, queue_size=1)

    def callback(self, data):
        with self.mutex:
            self.position = np.array([data.position.x, data.position.y, data.position.z])
            self.orientation = np.array([data.orientation.x, data.orientation.y, data.orientation.z])
            self.tags = data.tag_id.data
            self.tile = data.Quadrant.data

    def get_orientation(self):
        with self.mutex:
            return self.orientation

    def get_tile(self):
        with self.mutex:
            return self.tile
          
    def get_localization(self):
        with self.mutex:
            return self.position, self.orientation, self.tags, self.tile
    
    def get_tags(self):
        with self.mutex:
            return self.tags
          

class State(smach.State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker, outcomes, input_keys=[], output_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

        # Publishers And Subscribers
        self.rate = rospy.Rate(30)
        self.line_tracker: LineTracker = line_tracker
        self.motor_publisher: MotorController = motor_publisher
        self.tof_tracker: TofTracker = tof_tracker
        self.left_tracker: LeftTracker = left_tracker
        self.localization_tracker: LocalizationReader = localization_tracker

        # Local Variables
        self.current_tile = None
        self.offset = 75
    
    def drive(self, angularVelocity, linearVelocity=VELOCITY):
        self.motor_publisher.drive(angularVelocity, linearVelocity)
    
    def stop(self):
        self.motor_publisher.drive(0, 0)
    
    def track_line(self):
        centroid = self.line_tracker.get_line()
        omega = -(centroid + self.offset)/26
        if omega <= -3:
            omega = -3
        elif omega >= 3:
            omega = 3
        return omega

    def track_tof(self):
        return self.tof_tracker.get_distance()
    
    def get_localization(self):
        return self.localization_tracker.get_localization()
    
    def execute(self, ud):
        raise NotImplementedError


class DriveToTile(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker, outcomes=["reached_destination"], input_keys=["destination"])
        self.lane_changed = False
        self.saved_ticks = 0
        self.lane_change_time = 325
        self.min_dist = 0.27
        self.tof_history = deque(maxlen=MAX_TOF_QUEUE)
        self.prev_distance = 0
        self.tof_tolerence = 0.75

    def rotate_to_angle(self, target):
        """NOT TESTED, ROTATION MIGHT BE BACKWARDS"""
        deviation = ((target-self.localization_tracker.get_orientation()+180) % 360) - 180
        if deviation > 0:
            cw = -1 # False
        else:
            cw = 1  # True

        while abs(deviation) > self.rotation_error:
            self.drive(linearVelocity=0, angularVelocity=cw*4)  # Might be backwards
            self.rate.sleep()
            deviation = ((target-self.localization_tracker.get_orientation()+180) % 360) - 180

        return None

    def is_box_infront(self):
        if len([i for i in self.tof_history]) < MAX_TOF_QUEUE:
            return False
        total_diff = 0
        current_val = self.tof_history.popleft()
        for i in range(MAX_TOF_QUEUE-1):
            rospy.loginfo("QUEUE {}: = {}".format(i, current_val))
            second_val = self.tof_history.popleft()
            total_diff += current_val-second_val
            current_val = second_val
        if total_diff <= 0:
            return False

        rospy.loginfo("TOT. QUEUE DIFF: {}".format(total_diff))
        return total_diff < self.tof_tolerence

    def execute(self, ud):
        rospy.loginfo(f"Destination: {ud.destination}")
        while True:
            self.drive(-4, 0)
            rospy.sleep(0.5)
            self.drive(0, 0)
            rospy.sleep(1.0)
            rospy.loginfo(self.localization_tracker.get_tags())
        while self.track_line() == 0:
            self.rate.sleep()
        while self.localization_tracker.get_tile() != ud.destination:
            """
            # Put current tof into queue
            distance = self.tof_tracker.get_distance()
            if distance != self.prev_distance:
                self.tof_history.append(distance)
            self.prev_distance = distance

            #  If box in front, switch lanes and save current wheel encoder
            if distance < self.min_dist and self.lane_changed is False:
                if self.is_box_infront():
                    self.saved_ticks = self.left_tracker.get_left_ticks()
                    self.offset *= -1
                    self.lane_changed = True
                    rospy.loginfo("CHANGING TO LEFT LANE")

            # Drive for a little bit then switch back lanes
            if self.left_tracker.get_left_ticks() > (self.saved_ticks + self.lane_change_time) and self.lane_changed:
                rospy.loginfo("CHANGING TO RIGHT LANE")
                self.offset *= -1
                self.lane_changed = False
            """
            if self.line_tracker.get_line() != -999:
                self.drive(angularVelocity=self.track_line(), linearVelocity=VELOCITY)
            else:
                self.drive(angularVelocity=0, linearVelocity=VELOCITY)
            rospy.loginfo(self.localization_tracker.get_tile())
            self.rate.sleep()

        self.drive(0, 0)
        rospy.sleep(2)
        return "reached_destination"


class ChooseTile(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker, outcomes=["finish", "drive_to_tile"], output_keys=["destination"])
        self.map = Map()
        self.current_tile = "D5"
        self.path = []
        self.planner = Fonda()

    def execute(self, ud):
        if len(self.path) == 0:
            x = self.planner.get_intersection_from_tag("164")
            rospy.loginfo(x)
            self.path = self.map.get_path(self.current_tile, (x["intersection"], x["orient"]))
            rospy.loginfo(self.path)
        ud.destination = self.path.pop(0)
        rospy.loginfo(f"PATH: {self.path}")
        return "drive_to_tile"


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.motor_controller = MotorController()
        self.line_tracker = LineTracker()
        self.tof_tracker = TofTracker()
        self.left_tracker = LeftTracker()
        self.localization_tracker = LocalizationReader()
        
        self.sm = smach.StateMachine(outcomes=['FINISH'])

    def run(self):
        # Add states to the container
        with self.sm:
            smach.StateMachine.add('CHOOSE_TILE', ChooseTile(self.motor_controller, self.line_tracker, self.tof_tracker, self.left_tracker, self.localization_tracker), transitions={'finish':'FINISH', 'drive_to_tile':'DRIVE_TO_TILE'})
            smach.StateMachine.add('DRIVE_TO_TILE', DriveToTile(self.motor_controller, self.line_tracker, self.tof_tracker, self.left_tracker, self.localization_tracker), transitions={'reached_destination':'CHOOSE_TILE'})

        # Execute SMACH plan
        outcome = self.sm.execute()

        self.motor_controller.drive(0, 0)
        rospy.sleep(1)
        rospy.loginfo("FINISHED COMPETITION 2")

    def on_shutdown(self):
        """Shutdown procedure.
        - Publishes a zero velocity command at shutdown.
        """

        self.motor_controller.drive(0, 0)
        rospy.sleep(1)
        super(MyPublisherNode, self).on_shutdown()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='competition_2_main_node')
    node.run()
    rospy.spin()
