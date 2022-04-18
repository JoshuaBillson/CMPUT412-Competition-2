#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Int32
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import smach
from trackers import LineTracker, TofTracker, LeftTracker
from threading import Lock, Thread
from graph import Map
from collections import deque

HOSTNAME = "/" + os.uname()[1]
MOTOR_TOPIC = HOSTNAME + "/car_cmd_switch_node/cmd"
LOCALIZATION_TOPIC = HOSTNAME + "/output/localization"
VELOCITY = 0.40
MAX_TOF_QUEUE = 5

class MotorController:
    def __init__(self):
        self.pub = rospy.Publisher(MOTOR_TOPIC, Twist2DStamped, queue_size=1)
        #self.rate = rospy.Rate(30)
        self.offset = 100

        self.msg = Twist2DStamped()

    def drive(self, angularVelocity, linearVelocity):
        self.msg.v = linearVelocity
        self.msg.omega = -(angularVelocity + self.offset)/26
        self.pub.publish(self.msg)
        #self.rate.sleep() # Do we need this second sleep wouldn't the sleep in FollowPath handle this?  As drive is only
                          # ever executed once, it is not in any loop?

#class LocalizationReader:
#    def __init__(self):
#        self.x_pos = 0
#        self.y_pos = 0
#        self.tag_id = 0
#        self.mutex = Lock()
#        self.graph = MapGraph()
#        self.subscriber = rospy.Subscriber(LOCALIZATION_TOPIC, Int32, self.callback)
#
#    def callback(self, data):
#        with self.mutex:
#            self.tag_id = data.data
#            self.x_pos, self.y_pos = self.graph.get_coords(self.tag_id)
    

class State(smach.State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes, input_keys=[], output_keys=[]):
        self.rate = rospy.Rate(10)
        self.line_tracker: LineTracker = line_tracker
        self.motor_publisher: MotorController = motor_publisher
        self.tof_tracker: TofTracker = tof_tracker
        self.left_tracker: LeftTracker = left_tracker
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
    
    def drive(self, angularVelocity, linearVelocity=VELOCITY):
        self.motor_publisher.drive(angularVelocity, linearVelocity)
    
    def stop(self):
        self.motor_publisher.drive(0, 0)
    
    def track_line(self):
        return self.line_tracker.get_line()

    def track_tof(self):
        return self.tof_tracker.get_distance()
    
    def execute(self, ud):
        raise NotImplementedError


class FollowPath(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        self.tof_history = deque(maxlen=MAX_TOF_QUEUE)
        self.min_dist = 0.30
        self.lane_changed = False
        self.saved_ticks = 0
        self.lane_change_time = 325
        self.tof_tolerance = 1

        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["intersection", "obstacle"])

    def calc_queuediff(self):
        if len([i for i in self.tof_history]) < MAX_TOF_QUEUE:
            return 999
        total_diff = 0
        current_val = self.tof_history.popleft()
        for i in range(MAX_TOF_QUEUE-1):
            second_val = self.tof_history.popleft()
            total_diff += current_val-second_val
            current_val = second_val
        if total_diff < 0:
            return 999
        else:
            return total_diff

    def execute(self, ud):
        while self.track_line() == 0:
            self.rate.sleep()
        while not rospy.is_shutdown():
            # Put current tof into queue
            distance = self.tof_tracker.get_distance()
            self.tof_history.append(distance)

            #  If box in front, switch lanes and save current wheel encoder
            if distance < self.min_dist and self.lane_changed is False and self.calc_queuediff() < self.tof_tolerance:
                self.saved_ticks = self.left_tracker.get_left_ticks()
                self.motor_publisher.offset *= -1
                self.lane_changed = True
                rospy.loginfo("CHANGING TO LEFT LANE")

            # Drive for a little bit then switch back lanes
            if self.left_tracker.get_left_ticks() > (self.saved_ticks + self.lane_change_time) and self.lane_changed is True:
                rospy.loginfo("CHANGING TO RIGHT LANE")
                self.motor_publisher.offset *= -1
                self.motor_publisher.lane_changed = False
            self.drive(self.track_line())
            self.rate.sleep()

        return "intersection"


class ChooseIntersection(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["straight", "left", "right", "finished"])

    def execute(self, ud):
        pass


class TurnLeft(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class TurnRight(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class TurnStraight(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class AvoidObstacle(State):
    def __init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker):
        State.__init__(self, motor_publisher, line_tracker, tof_tracker, left_tracker, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.motor_controller = MotorController()
        self.line_tracker = LineTracker()
        self.tof_tracker = TofTracker()
        self.left_tracker = LeftTracker()
        self.sm = smach.StateMachine(outcomes=['FINISH'])
        #self.pub = rospy.Publisher(TOPIC, String, queue_size=10)

    def run(self):
        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('FOLLOW_PATH', FollowPath(self.motor_controller,
                                                             self.line_tracker,
                                                             self.tof_tracker,
                                                             self.left_tracker),
                                   transitions={'intersection':'CHOOSE_INTERSECTION', 'obstacle':'AVOID_OBSTACLE'})
            smach.StateMachine.add('CHOOSE_INTERSECTION', ChooseIntersection(self.motor_controller,
                                                                             self.line_tracker,
                                                                             self.tof_tracker,
                                                                             self.left_tracker),
                                   transitions={'finished':'FINISH', "left": "LEFT", "right": "RIGHT", "straight": "STRAIGHT"})
            smach.StateMachine.add('LEFT', TurnLeft(self.motor_controller,
                                                    self.line_tracker,
                                                    self.tof_tracker,
                                                    self.left_tracker),
                                   transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('RIGHT', TurnRight(self.motor_controller,
                                                      self.line_tracker,
                                                      self.tof_tracker,
                                                      self.left_tracker),
                                   transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('STRAIGHT', TurnStraight(self.motor_controller,
                                                            self.line_tracker,
                                                            self.tof_tracker,
                                                            self.left_tracker),
                                   transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('AVOID_OBSTACLE', AvoidObstacle(self.motor_controller,
                                                                   self.line_tracker,
                                                                   self.tof_tracker,
                                                                   self.left_tracker),
                                   transitions={'follow-path':'FOLLOW_PATH'})

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
