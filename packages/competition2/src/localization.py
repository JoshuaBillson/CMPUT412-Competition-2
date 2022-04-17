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
from dt_apriltags import Detector
from duckietown.dtros import DTROS, NodeType


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # April Tag Thread
        self.thread = Thread(target=self.detect_loop)
        self.mutex = Lock()
        self.img = None

        # Initialize Detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tagStandard41h12',
                                    nthreads=1,
                                    quad_decimate=1.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)
     
        # Add your subsribers or publishers here
        self.subscriber = rospy.Subscriber("/csc22912/camera_node/image/compressed", CompressedImage, self.imgCallback, queue_size=1)

        # Add information about tags
        TAG_SIZE = .08
        FAMILIES = "tagStandard41h12"
        self.tags = Tag(TAG_SIZE, FAMILIES)
        # (tag id, x, y, z, thetax, thetay, thetaz, type)
        self.tags.add_tag(8, 1.74, 0, 2.93, 0, -math.pi / 2, 0, "generic") 
        self.tags.add_tag(13, 1.47, 0, 1.82, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(14, 1.22, 0, 0.635, 0, 0, 0, "3wayleft")
        self.tags.add_tag(15, 2.535, 0, 0.045, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(21, 1.815, 0, 1.48, 0, 0, 0, "generic") 
        self.tags.add_tag(22, 2.08, 0, 0.56, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(23, 2.925, 0, 1.82, 0, math.pi, 0, "generic")
        self.tags.add_tag(24, 1.12, 0, 2.93, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(35, 1.74, 0, 1.155, 0, math.pi, 0, "3wayright") 
        self.tags.add_tag(36, 0.56, 0, 2.335, 0, math.pi, 0, "3wayright")
        self.tags.add_tag(37, 1.22, 0, 0.56, 0, -math.pi / 2, 0, "3wayright")
        self.tags.add_tag(38, 0.045, 0, 1.75, 0, -math.pi / 2, 0, "3waytee")
        self.tags.add_tag(44, 2.335, 0, 2.335, 0, math.pi, 0, "4way")
        self.tags.add_tag(45, 2.335, 0, 1.82, 0, math.pi / 2, 0, "4way")
        self.tags.add_tag(46, 1.815, 0, 2.335, 0, -math.pi / 2, 0, "4way")
        self.tags.add_tag(47, 1.815, 0, 1.82, 0, 0, 0, "4way")
        self.tags.add_tag(59, 0.425, 0, 2.93, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(63, 0.56, 0, 0.56, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(67, 0.88, 0, 1.225, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(75, 0.87, 0, 1.82, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(76, 0.045, 0, 0.605, 0, 0, 0, "generic")
        self.tags.add_tag(77, 2.55, 0, 1.505, 0, 0, 0, "generic")
        self.tags.add_tag(78, 1.115, 0, 1.75, 0, -math.pi / 2, 0, "generic")
        self.tags.add_tag(89, 1.815, 0, 1.155, 0, -math.pi / 2, 0, "3wayright")
        self.tags.add_tag(90, 2.925, 0, 1.155, 0, math.pi, 0, "3wayleft")
        self.tags.add_tag(91, 2.335, 0, 0.635, 0, math.pi / 2, 0, "3wayleft")
        self.tags.add_tag(92, 1.815, 0, 0.635, 0, 0, 0, "3waytee")
        self.tags.add_tag(164, 0.965, 0, 2.41, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(166, 2.925, 0, 0.635, 0, math.pi / 2, 0, "3waytee")
        self.tags.add_tag(190, 1.59, 0, 2.41, 0, 0, math.pi / 2, "generic")
        self.tags.add_tag(191, 0.635, 0, 0.045, 0, math.pi / 2, 0, "generic")
        self.tags.add_tag(192, 0.045, 0, 2.335, 0, -math.pi / 2, 0, "3waytee")
        self.tags.add_tag(205, 1.22, 0, 0.045, 0, 0, 0, "3waytee")
        self.tags.add_tag(206, 1.74, 0, 0.045, 0, math.pi / 2, 0, "3wayleft")
        self.tags.add_tag(207, 1.22, 0, 1.155, 0, -math.pi / 2, 0, "3waytee")
        self.tags.add_tag(226, 0.045, 0, 1.82, 0, 0, 0, "3wayleft")
        self.tags.add_tag(227, 0.045, 0, 1.225, 0, 0, 0, "3wayleft")
        self.tags.add_tag(228, 0.56, 0, 1.75, 0, 0, math.pi, "3wayright")
        self.tags.add_tag(303, 2.415, 0, 0.635, 0, 0, 0, "3wayright")
        self.tags.add_tag(304, 1.155, 0, 2.335, 0, -math.pi / 2, 0, "generic")


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
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        # Extract Tags From Image
        undistorted_image = self.undistort(image_np)
        grayscale_image = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2GRAY)
        with self.mutex:
            self.img = np.copy(grayscale_image)

    def detect_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.mutex:
                img = np.copy(self.img)
            if img is not None and len(img.shape) == 2:
                tags = self.detect(img)
                if len(tags) > 0:
                    tag = tags[0]
                    self.tags.estimate_pose(tag.tag_id, tag.pose_R, tag.pose_t)
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
        
        rospy.loginfo(f"Detect Image: {img.shape}")

        return self.at_detector.detect(img, estimate_tag_pose=True, camera_params=PARAMS, tag_size=0.08)


def main():
    node = LocalizationNode(node_name='competition_2_localization_node')
    node.run()


if __name__ == "__main__":
    main()
