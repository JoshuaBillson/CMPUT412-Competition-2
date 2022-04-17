#!/usr/bin/env python3

import os
from typing_extensions import Self
import numpy as np
import yaml
import cv2
import math
import os
import rospy
from threading import Thread, Lock
from tag import Tag
from sensor_msgs.msg import CompressedImage
from competition2.msg import Localization
from geometry_msgs.msg import Quaternion, Point, Pose, Vector3
from std_msgs.msg import Int32, String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType


HOSTNAME = "/" + os.uname()[1]
LOCATION_TOPIC = HOSTNAME + "/output/location"


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # April Tag Thread
        self.thread = Thread(target=self.detect_loop)
        self.mutex = Lock()
        self.img = None
        self.location = np.array([[0], [0], [0]])
        self.rotation = np.array([[0], [0], [0]])

        # Initialize Detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
     
        # Add your subsribers or publishers here
        self.subscriber = rospy.Subscriber("/csc22912/camera_node/image/compressed", CompressedImage, self.imgCallback, queue_size=1)
        self.location_publisher = rospy.Publisher(LOCATION_TOPIC, Localization, queue_size=10)

        # Add information about tags
        TAG_SIZE = .08
        FAMILIES = "tagStandard41h12"
        self.tags = Tag(TAG_SIZE, FAMILIES)
        self.tags.add_tag(8, 1.74, 0, 2.93, 0, math.pi / 2, 0, "generic") 
        self.tags.add_tag(13, 1.47, 0, 1.82, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(14, 1.22, 0, 0.635, 0, math.pi, 0, "3wayleft")
        self.tags.add_tag(15, 2.535, 0, 0.045, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(21, 1.815, 0, 1.48, 0, math.pi, 0, "generic") 
        self.tags.add_tag(22, 2.08, 0, 0.56, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(23, 2.925, 0, 1.82, 0, 0, 0, "generic")
        self.tags.add_tag(24, 1.12, 0, 2.93, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(35, 1.74, 0, 1.155, 0, 0, 0, "3wayright") 
        self.tags.add_tag(36, 0.56, 0, 2.335, 0, 0, 0, "3wayright")
        self.tags.add_tag(37, 1.22, 0, 0.56, 0, math.pi / 2, 0, "3wayright")
        self.tags.add_tag(38, 0.045, 0, 1.75, 0, math.pi / 2, 0, "3waytee")
        self.tags.add_tag(44, 2.335, 0, 2.335, 0, 0, 0, "4way")
        self.tags.add_tag(45, 2.335, 0, 1.82, 0, -math.pi / 2, 0, "4way")
        self.tags.add_tag(46, 1.815, 0, 2.335, 0, math.pi / 2, 0, "4way")
        self.tags.add_tag(47, 1.815, 0, 1.82, 0, math.pi, 0, "4way")
        self.tags.add_tag(59, 0.425, 0, 2.93, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(63, 0.56, 0, 0.56, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(67, 0.88, 0, 1.225, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(75, 0.87, 0, 1.82, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(76, 0.045, 0, 0.605, 0, math.pi, 0, "generic")
        self.tags.add_tag(77, 2.55, 0, 1.505, 0, math.pi, 0, "generic")
        self.tags.add_tag(78, 1.115, 0, 1.75, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(89, 1.815, 0, 1.155, 0, math.pi / 2, 0, "3wayright")
        self.tags.add_tag(90, 2.925, 0, 1.155, 0, 0, 0, "3wayleft")
        self.tags.add_tag(91, 2.335, 0, 0.635, 0, -math.pi / 2, 0, "3wayleft")
        self.tags.add_tag(92, 1.815, 0, 0.635, 0, math.pi, 0, "3waytee")
        self.tags.add_tag(164, 0.965, 0, 2.41, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(166, 2.925, 0, 0.635, 0, -math.pi / 2, 0, "3waytee")
        self.tags.add_tag(190, 1.59, 0, 2.41, 0, 0, -math.pi / 2, "generic")
        self.tags.add_tag(191, 0.635, 0, 0.045, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(192, 0.045, 0, 2.335, 0, math.pi / 2, 0, "3waytee")
        self.tags.add_tag(205, 1.22, 0, 0.045, 0, math.pi, 0, "3waytee")
        self.tags.add_tag(206, 1.74, 0, 0.045, 0, -math.pi / 2, 0, "3wayleft")
        self.tags.add_tag(207, 1.22, 0, 1.155, 0, math.pi / 2, 0, "3waytee")
        self.tags.add_tag(226, 0.045, 0, 1.82, 0, math.pi, 0, "3wayleft")
        self.tags.add_tag(227, 0.045, 0, 1.225, 0, math.pi, 0, "3wayleft")
        self.tags.add_tag(228, 0.56, 0, 1.75, 0, 0, 0, "3wayright")
        self.tags.add_tag(303, 2.415, 0, 0.635, 0, math.pi, 0, "3wayright")
        self.tags.add_tag(304, 1.155, 0, 2.335, 0, math.pi / 2, 0, "generic")

        # Load camera parameters
        with open("/data/config/calibrations/camera_intrinsic/" + os.uname()[1] + ".yaml") as file:
                camera_list = yaml.load(file,Loader = yaml.FullLoader)

        self.camera_intrinsic_matrix = np.array(camera_list['camera_matrix']['data']).reshape(3,3)
        self.distortion_coeff = np.array(camera_list['distortion_coefficients']['data']).reshape(5,1)
    
    def imgCallback(self, ros_data):
        '''
        This Callback Runs Whenever A New Image Is Published From The Camera.
        We Use This To Detect The Centroid Of The Path As Well As Its Colour.
        '''
        #### direct conversion to CV2 ####
        with self.mutex:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    def detect_loop(self):
        """Loop To Detect Location From Current Camera Image"""
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            
            msg = Localization()
            # Copy The Camera Image
            if self.img is not None:
                with self.mutex:
                    img = np.copy(self.img)
            else:
                img = None
        
            # Process Image
            if img is not None:

                # Prepare Image For Processing
                undistorted_image = self.undistort(img)
                grayscale_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)

                # Detect Tags
                count = 0
                tags = self.detect(grayscale_image)
                location = np.array([[0.0], [0.0], [0.0]])
                rotation = np.array([[0.0], [0.0], [0.0]])

                # For Each Detected Tag, Find The Global Coordinates And Take The Average
                for tag in tags:
                    l, r = self.tags.estimate_pose(tag.tag_id, tag.pose_R, tag.pose_t)
                    location += l
                    rotation += r
                    count += 1

                # Publish Tag ID
                if len(tags) > 0:
                    tag = tags[0]
                    tag_id_msg = Int32(tag.tag_id)
                    msg.tag_id = tag_id_msg

                # If No Tags Were Detected, The Location Does Not Change
                self.location = location / count if count > 0 else self.location
                self.rotation = rotation / count if count > 0 else self.rotation

            # Publish Pose And Tag
            position_msg = Vector3()
            position_msg.x = self.location[0,0]
            position_msg.y = self.location[1,0]
            position_msg.z = self.location[2,0]
            msg.position = position_msg

            # Publish Orientation
            orientation_msg = Vector3()
            orientation_msg.x = self.rotation[0,0]
            orientation_msg.y = self.rotation[1,0]
            orientation_msg.z = self.rotation[2,0]
            msg.orientation = orientation_msg

            # Publish current tile location
            tile_msg = String()
            xgrid = math.ceil(round(self.location[0,0],1)/0.6)
            zgrid = math.floor(round(self.location[2,0],1)/0.6)
            gridletters = ['E','D','C','B','A']
            zgrid_corr = gridletters[zgrid]
            current_tile = f"{zgrid_corr}{xgrid}"
            tile_msg = String(current_tile)
            msg.Quadrant = tile_msg

            #rospy.loginfo(msg)
            self.location_publisher.publish(msg)

            #rospy.loginfo(f"Rotation: ({self.rotation[0,0]}, {self.rotation[1,0]}, {self.rotation[2,0]})")
            #rospy.loginfo(f"Location: ({self.location[0,0]}, {self.location[1,0]}, {self.location[2,0]})")
            rate.sleep()

    def run(self):
        self.thread.start()
        rospy.spin()
        self.thread.join()

    def undistort(self, img):
        '''
        Takes a fisheye-distorted image and undistorts it
        Adapted from: https://github.com/asvath/SLAMDuck
        '''
        height = img.shape[0]
        width = img.shape[1]

        newmatrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_intrinsic_matrix,
            self.distortion_coeff, 
            (width, height),
            1, 
            (width, height))

        map_x, map_y = cv2.initUndistortRectifyMap(
            self.camera_intrinsic_matrix, 
            self.distortion_coeff,  
            np.eye(3), 
            newmatrix, 
            (width, height), 
            cv2.CV_16SC2)

        undistorted_image = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)
       
        return undistorted_image   
             

    def detect(self, img):
        ''' Takes an images and detects AprilTags '''
        PARAMS = [
            self.camera_intrinsic_matrix[0,0],
            self.camera_intrinsic_matrix[1,1],
            self.camera_intrinsic_matrix[0,2],
            self.camera_intrinsic_matrix[1,2]] 
        
        return self.at_detector.detect(img, estimate_tag_pose=True, camera_params=PARAMS, tag_size=0.065)


def main():
    node = LocalizationNode(node_name='lab_5_localization_node')
    node.run()


if __name__ == "__main__":
    main()
