#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import os
from threading import Lock

road_mask = [(20,60,0), (50,255,255)]
HOSTNAME = "/" + os.uname()[1]

class Camera(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Camera, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        #self.img_pub = rospy.Publisher("/output/image/compressed",
        #                           CompressedImage,
        #                           queue_size= 1)
        self.sub = rospy.Subscriber(HOSTNAME + "/camera_node/image/compressed",
                                    CompressedImage,
                                    self.img_callback,
                                    queue_size=1)
        self.pub = rospy.Publisher("/output/line_tracker",
                                   Float32,
                                   queue_size=1)
        self.np_arr = None
        self.msg = Float32()
        self.mutex = Lock()
        self.rate = rospy.Rate(30)

    def img_callback(self, ros_data):
        with self.mutex:
            self.np_arr = np.frombuffer(ros_data.data, np.uint8)

    def run(self):
        while not rospy.is_shutdown():
            if self.np_arr is not None:
                with self.mutex:
                    image = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
                height = image.shape[0]
                width = image.shape[1]
                crop = image[height-150:height-20,
                       int(width/2)-200:int(width/2)+200]
                crop_width = crop.shape[1]
                hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, road_mask[0], road_mask[1])
                contours, hierarchy = cv2.findContours(mask,
                                                       cv2.RETR_EXTERNAL,
                                                       cv2.CHAIN_APPROX_NONE)
                max_idx = -1
                max_area = 0
                centroid = -1

                # Search for lane in front / find max area
                for i in range(len(contours)):
                    area = cv2.contourArea(contours[i])
                    if area > max_area:
                        max_idx = i
                        max_area = area

                if max_idx != -1:
                    M = cv2.moments(contours[max_idx])
                    cx = int(M['m10']/M['m00'])
                    centroid = cx - int(crop_width/2)

                self.msg.data = centroid
                self.pub.publish(centroid)
            else:
                pass

            self.rate.sleep()


if __name__ == '__main__':
    # create the node
    node = Camera(node_name='camera_node')
    node.run()
    # run node
    rospy.spin()
