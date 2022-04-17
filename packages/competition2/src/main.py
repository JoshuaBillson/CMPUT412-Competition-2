#!/usr/bin/env python3

import time
import os
import rospy
from std_msgs.msg import Float32, Int32, String
from duckietown.dtros import NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String
import smach
from threading import Lock, Thread
from graph import MapGraph

HOSTNAME = "/" + os.uname()[1]
MOTOR_TOPIC = HOSTNAME + "/car_cmd_switch_node/cmd"
LINE_TRACKER_TOPIC = HOSTNAME + "/output/line_tracker"
LOCALIZATION_TOPIC = HOSTNAME + "/output/localization"
VELOCITY = 0.30

class MotorController:
    def __init__(self):
        self.pub = rospy.Publisher(MOTOR_TOPIC, Twist2DStamped, queue_size=10)
        self.rate = rospy.Rate(10)
    
    
    def drive(self, linearVelocity, angularVelocity):
        pass
    

class LineTracker:
    def __init__(self):
        self.line = 0
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(LINE_TRACKER_TOPIC, Float32, self.callback)
    
    def callback(self, data):
        with self.mutex:
            self.line = data.data
    
    def get_line(self):
        with self.mutex:
            return self.line


class LocalizationReader:
    def __init__(self):
        self.x_pos = 0
        self.y_pos = 0
        self.tag_id = 0
        self.mutex = Lock()
        self.graph = MapGraph()
        self.subscriber = rospy.Subscriber(LOCALIZATION_TOPIC, Int32, self.callback)
    
    def callback(self, data):
        with self.mutex:
            self.tag_id = data.data
            self.x_pos, self.y_pos = self.graph.get_coords(self.tag_id)
    

class State(smach.State):
    def __init__(self, motor_publisher, line_tracker, outcomes, input_keys=[], output_keys=[]):
        self.rate = rospy.Rate(30)
        self.line_tracker: LineTracker = line_tracker
        self.motor_publisher: MotorController = motor_publisher
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
    
    def drive(self, linearVelocity, angularVelocity):
        self.motor_publisher.drive(linearVelocity, angularVelocity)
    
    def stop(self):
        self.motor_publisher.drive(0, 0)
    
    def track_line(self):
        return self.line_tracker.get_line()
    
    def is_stop_sign(self):
        return self.stop_matcher.stop_sign_detected()
    
    def execute(self, ud):
        raise NotImplementedError


class FollowPath(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["intersection", "obstacle"])

    def execute(self, ud):
        while self.track_line() == 0:
            self.rate.sleep()
        while not self.is_stop_sign():
            self.drive(VELOCITY, self.track_line())
            self.rate.sleep()
        return "intersection"


class ChooseIntersection(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["straight", "left", "right", "finished"])

    def execute(self, ud):
        pass


class TurnLeft(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class TurnRight(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class TurnStraight(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class AvoidObstacle(State):
    def __init__(self, motor_publisher, line_tracker, stop_matcher):
        State.__init__(self, motor_publisher, line_tracker, stop_matcher, outcomes=["follow-path"])

    def execute(self, ud):
        pass


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(TOPIC, String, queue_size=10)

    def run(self):

        motor_controller = MotorController()
        line_tracker = LineTracker()
        sm = smach.StateMachine(outcomes=['FINISH'])

    
        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('FOLLOW_PATH', FollowPath(motor_controller, line_tracker), transitions={'intersection':'CHOOSE_INTERSECTION', 'obstacle':'AVOID_OBSTACLE'})
            smach.StateMachine.add('CHOOSE_INTERSECTION', ChooseIntersection(motor_controller, line_tracker), transitions={'finish':'FINISH', "left": "LEFT", "right": "RIGHT", "straight": "STRAIGHT"})
            smach.StateMachine.add('LEFT', TurnLeft(motor_controller, line_tracker), transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('RIGHT', TurnRight(motor_controller, line_tracker), transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('STRAIGHT', TurnStraight(motor_controller, line_tracker), transitions={'follow-path':'FOLLOW_PATH'})
            smach.StateMachine.add('AVOID_OBSTACLE', AvoidObstacle(motor_controller, line_tracker), transitions={'follow-path':'FOLLOW_PATH'})
    
        # Execute SMACH placompetitionn
        outcome = sm.execute()

        motor_controller.drive(0, 0)
        rospy.sleep(1)
        rospy.loginfo("FINISHED COMPETITION 2")

    def on_shutdown(self):
        """Shutdown procedure.
        - Publishes a zero velocity command at shutdown.
        """

        #TODO KILL MOTORS

        super(MyPublisherNode, self).on_shutdown()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='competition_2_main_node')
    node.run()
