#!/usr/bin/env python3

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
VELOCITY = 0.40
MAX_TOF_QUEUE = 5

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
            self.tags = [data.tag_id.data]
            self.tile = data.Quadrant.data

    def get_localization(self):
        with self.mutex:
            return self.position, self.orientation, self.tags, self.tile
   

class State(smach.State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker, outcomes, input_keys=[], output_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)

        # Publishers And Subscribers
        self.rate = rospy.Rate(10)
        self.line_tracker: LineTracker = line_tracker
        self.motor_publisher: MotorController = motor_publisher
        self.tof_tracker: TofTracker = tof_tracker
        self.left_tracker: LeftTracker = left_tracker
        self.localization_tracker: LocalizationReader = localization_tracker

        # Local Variables
        self.current_tile = None
        self.offset = 125
    
    def drive(self, angularVelocity, linearVelocity=VELOCITY):
        self.motor_publisher.drive(angularVelocity, linearVelocity)
    
    def stop(self):
        self.motor_publisher.drive(0, 0)
    
    def track_line(self):
        return -(self.line_tracker.get_line() + self.offset) / 26

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
        self.min_distance = 0.27
        self.tof_history = deque(maxlen=MAX_TOF_QUEUE)
        self.prev_distance = 0
        self.tof_tolerence = 0.75
        
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
        while self.track_line() == 0:
            self.rate.sleep()
        while not rospy.is_shutdown():
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
                self.motor_publisher.offset *= -1
                self.motor_publisher.lane_changed = False
            self.drive(angularVelocity=self.track_line(), linearVelocity=VELOCITY)
            self.rate.sleep()

        return "intersection"


class ChooseTile(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, localization_tracker, outcomes=["finish", "drive_to_tile"], output_keys=["destination"])
        self.map = Map()

    def execute(self, ud):
        counter = 0
        while counter < 100:
            position, orientation, tag, tile = self.get_localization()
            rospy.loginfo(f"Position: {position}  Orientation: {orientation}  Tile: {tile}  Counter: {counter}")
            self.drive(0, 0)
            self.rate.sleep()
            counter += 1
        ud.destination = "A5"
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
