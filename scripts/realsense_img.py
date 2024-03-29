#!/usr/bin/env python
from typing import Any
import pyrealsense2 as rs
import numpy as np
import cv2
import pandas as pd
# from mapping import ArucoMapping
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from aruco_detec_pose_estimate import aruco_pose


class ros_camera:
    real_camera_img_aligned = None
    
    def __init__(self, node_name='image'):

        rospy.init_node(node_name, anonymous=False)
    
    def camera_imag(self):
        # rospy.init_node("image", anonymous=False)
        camera_topic = "/camera/color/image_raw"
        rospy.Subscriber(camera_topic, Image, self.camera_aligned_callback,queue_size=5)
        
    def camera_aligned_callback(self, msg):
        # global real_camera_img_aligned
        self.real_camera_img_aligned = msg
        
        
    def get_image(self):
        bridge = CvBridge()
        image = self.real_camera_img_aligned
        while True:
            self.camera_imag()
            image = self.real_camera_img_aligned
            # print('imga')
            # print(image)
            if image is not None:
                # image_np = np.array(image.data, dtype=np.uint8)
                # print('1')
                cv_img = bridge.imgmsg_to_cv2(image,  desired_encoding="bgr8")
                return cv_img

            else:
                continue
            
    def display(self, collect=False):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        aruco_dict2 = aruco.Dictionary_get(aruco.DICT_6X6_250)
        aruco_dict3 = aruco.Dictionary_get(aruco.DICT_5X5_250)
        aruco_dict4 = aruco.Dictionary_get(aruco.DICT_4X4_250)
        arucopose = aruco_pose()
        
        filename = "endpose_data.csv"
        # filename = "endpose_aruco.csv"
        # img_path = './checkerboard_img/'
        # img_path = './aruco_img/'
        img_path = './charuco_test/'
        epi = 0
        
        while True:
            image = self.get_image()
            if collect is False:
                arucopose.PoseEstimate(image, aruco_dict, draw=True)
                # arucopose.PoseEstimate(image, aruco_dict2, aruco_size=0.022, draw=True)
                arucopose.PoseEstimate(image, aruco_dict3, aruco_size=0.007, draw=True, axis_size=0.004)
                arucopose.PoseEstimate(image, aruco_dict4, aruco_size=0.007, draw=True, axis_size=0.004)

            cv2.imshow('RealSense', image)
            key =cv2.waitKey(1)
            if key==ord('q'):
            
                break
            if (key == ord('s') or key == ord('S')):
                from UR_env import Ur5_env
                robot_control = Ur5_env()
                epi+=1
                imgname = str(epi) + ".jpg"
                end_position, end_orientation = robot_control.info_get()
                print(epi)
                cv2.imwrite(img_path+imgname, image)
   
                df = pd.DataFrame([end_position + end_orientation], columns=['position x', 'position y', 'position z', 'Rotdata x', 'Rotdata y', 'Rotdata z', 'Rotdata w'])     # np.concatenate((list_tr, list_rot), axis=1)
                if epi != 1:

                    df_old = pd.read_csv(img_path +filename)
                    # Concatenate the existing DataFrame with the new DataFrame
                    df = pd.concat([df_old, df], ignore_index=True)

                # Save the updated DataFrame back to the file
                df.to_csv(img_path+filename, index=False)


if __name__== "__main__":
    
    rl_camera = ros_camera()
    print("If it's used for calibration, please enter 1, or 0")
    user_input = input("Enter 1 or 0:")

    if user_input == "1":
        print("press 's' to save image")
        pur = True
    elif user_input == "0":
        # print("Goodbye!")
        pur = False
    else:
        print("Please re-launch if you plan to calibrate")
        pur = False
    print("press 'q' to exit")
    rl_camera.display(collect=pur)
