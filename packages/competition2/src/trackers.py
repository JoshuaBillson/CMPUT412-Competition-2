import rospy
from threading import Lock
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelEncoderStamped

import os

HOSTNAME = os.environ['DUCKNAME']
LINE_TRACKER_TOPIC = "/output/line_tracker"
TOF_TRACKER_TOPIC = HOSTNAME + "/front_center_tof_driver_node/range"
LEFT_TRACKER_TOPIC = HOSTNAME + "/left_wheel_encoder_node/tick"
RIGHT_TRACKER_TOPIC = HOSTNAME + "/right_wheel_encoder_node/tick"


class LineTracker:
    def __init__(self):
        self.line = 0
        self.offset = 30
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(LINE_TRACKER_TOPIC, Float32, self.callback)

    def callback(self, data):
        if data.data != -1:
            with self.mutex:
                self.line = data.data

    def get_line(self):
        with self.mutex:
            return self.line


class TofTracker:
    def __init__(self):
        self.distance = 0
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(TOF_TRACKER_TOPIC, Range, self.callback)

    def callback(self, data):
        with self.mutex:
            self.distance = data.range

    def get_distance(self):
        with self.mutex:
            return self.distance


class LeftTracker:
    def __init__(self):
        self.left_ticks = 0
        self.mutex = Lock()
        self.subscriber = rospy.Subscriber(LEFT_TRACKER_TOPIC, WheelEncoderStamped, self.callback)

    def callback(self, data):
        with self.mutex:
            self.left_ticks = data.data

    def get_left_ticks(self):
        with self.mutex:
            return self.left_ticks
