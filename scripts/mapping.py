#!/usr/bin/env python
from typing import Any
import pyrealsense2 as rs
import numpy as np
import cv2
import imutils
import cv2.aruco as aruco
from Translation import Transformation
from UR_env import Ur5_env
import time
import threading
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation
import os
from realsense_img import ros_camera
from aruco_detec_pose_estimate import aruco_pose
from moveit_python.planning_scene_interface import PlanningSceneInterface
from general_fun import table_cal
import transforms3d as tfs


class ArucoMapping:
    # no collusion, calculate cuboid from 4 points
    def cuboid_geometry(self, tr_f, tr_b, name, label):
        rectangle_calculate = table_cal()
        # get 4 points
        # label = 
        rl, rr = rectangle_calculate.corner_points(tr_f, label[0])
        ll, lr = rectangle_calculate.corner_points(tr_b, label[1])#140
        
        
        wide1 = rectangle_calculate.calculate_distance(rl, rr)
        wide2 = rectangle_calculate.calculate_distance(ll, lr)
        width = (wide1+wide2)/2
        
        len1 = rectangle_calculate.calculate_distance(rr, lr)
        len2 = rectangle_calculate.calculate_distance(rl, ll)
        length = (len1+len2)/2
        points = [rl, rr, ll, lr]
        center = rectangle_calculate.calculate_center(points)
        
        height = 0.04       #defualt
        
        # calculate the orientation of the table
        
        P_f, V_r = rectangle_calculate.line_equation(rl,rr)            # right line
        P_r, V_b = rectangle_calculate.line_equation(rr, lr)           # bottom line
        normal_vector = np.cross(V_r, V_b)
        
        rotation_matrix = np.column_stack((V_r, V_b, normal_vector))
        print('V_f', V_r, '\nV_b', V_b, '\nnormal_vector:', normal_vector)
        # Convert the rotation matrix to a quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
    # Normalize quaternion
        quaternion /= np.linalg.norm(quaternion)
        print('quaternion:', quaternion)
        print('wide:', width, 'length:', length, 'center on the surface:', center)
        # filename = "object_data.csv"
        row_name = name #'table'
        
        center[2] -= height/2
        csv_row = {}

        csv_row[row_name] = [row_name, width, length, height, center[0], center[1], center[2]]
        
        ArucoMapping.csv_save(csv_row)
        
        # #===== add collision object====
        cuboid_size = [width, length, 0.01]
        ArucoMapping.collision_obj(center, quaternion, cuboid_size, row_name)
        
        return rl, rr, ll, lr
    
    @staticmethod
    def world_base_trans(position, orien, num):
        trans = Transformation()
        tr = [0, 0, 0]
        if num == 0:
            #transfer pose in base link to dummy
            quat_base = np.array([0.92388, 0, 0, 0.382683])
            
        elif num ==1:
            # transfer pose in dummy to base_link
            quat_base = np.array([0.92388, 0, 0, -0.382683])
            
        base_rot = trans.RotMatrix(quat_base)
        H_d2b = tfs.affines.compose(np.squeeze(tr), base_rot, [1, 1, 1])                 # dummy to base
        obj_rot = trans.RotMatrix(orien)
        H_b2o = tfs.affines.compose(np.squeeze(position), obj_rot, [1, 1, 1])                 # dummy to base
            
        H_b2o = np.dot(H_d2b,H_b2o)   
        translation = H_b2o[0:3,3:4].T[0]

        quat = tfs.quaternions.mat2quat(H_b2o[0:3,0:3])                  # wxyz form
        quat_t = np.roll(quat, -1)
            
        return translation, quat_t

    
    
    def base_geometry(self, tr_f, tr_b, name, label, refer_qua, direction):
        rectangle_calculate = table_cal()
        # get 4 points
        # label = 
        rl, rr = rectangle_calculate.corner_points(tr_f, label[0])
        ll, lr = rectangle_calculate.corner_points(tr_b, label[1])#140
        
        
        wide1 = rectangle_calculate.calculate_distance(rl, rr)
        wide2 = rectangle_calculate.calculate_distance(ll, lr)
        width = (wide1+wide2)/2 #+0.02
        
        len1 = rectangle_calculate.calculate_distance(rr, lr)
        len2 = rectangle_calculate.calculate_distance(rl, ll)
        length = (len1+len2)/2 #+ (0.02)
        points = [rl, rr, ll, lr]
        center = rectangle_calculate.calculate_center(points)
        
        height = 0.01       #defualt
        
        
        row_name = name #'table'
        # tr, quat = ArucoMapping.world_base_trans(center, refer_qua, 0)
        # if direction == 'vertical':
            
        #     tr[0] -= height/2
   
        # elif direction == 'horizontal':
        #     tr[2] -= height/2
            
        # center, refer_qua = ArucoMapping.world_base_trans(tr, quat, 1)
        csv_row = {}

        csv_row[row_name] = [row_name, width, length, height, center[0], center[1], center[2]]
        
        ArucoMapping.csv_save(csv_row)
        
        # #===== add collision object====
        cuboid_size = [length, width, height]
        ArucoMapping.collision_obj(center, refer_qua, cuboid_size, row_name, obj_color=[0.0, 1.0, 1.0])
        
    def singal_aruco_object(self, translation, id, size, name, direction):
        position = translation[id][0]
        orien = translation[id][1]
        
        tr, quat = ArucoMapping.world_base_trans(position, orien, 0)
        print('z test before:', tr)
        tr_test = tr
        tr_test[2] += 0.06
        center, _ = ArucoMapping.world_base_trans(tr_test, quat, 1)
        print('z test after:', center)
        if direction == 'vertical':
            print('tr', tr)
            tr[0] -= size[2]/2
            tr[2] += 0.06
            # tr = tr
            print('tr_after', tr)
            
   
        elif direction == 'horizontal':
            tr[2] -= size[2]/2
            
        elif direction == 'special':
            tr[0] += size[2]/2      # aruco marker is on the bottom of the object
            
        center, _ = ArucoMapping.world_base_trans(tr, quat, 1)
        # center = position
        ArucoMapping.collision_obj(center, orien, size, name, obj_color=[0.0, 0.0, 1.0])


        
    
    @staticmethod
    def csv_save(csv_row):
        filename = "object_data.csv"
        collision_path = './Collision_object/'
        column_name = ['name', 'width', 'length', 'height', 'center x', 'center y', 'center z']
        if os.path.isfile(collision_path+filename):
            df_old = pd.read_csv(collision_path+filename)

            for idx in csv_row.keys():
                # print('idx:', idx)
                # print('df_old:', df_old['name'])
                if idx in df_old['name'].values:
                    df_old.loc[df_old['name'] == idx] = csv_row[idx]
                else:

                    df = pd.DataFrame([csv_row[idx]], columns=column_name)
                    df_old = pd.concat([df_old, df])
            df = df_old
        else:
            df = pd.DataFrame(csv_row.values(), columns=column_name)

        df.to_csv(collision_path+filename, index=False)
        

    @staticmethod
    def collision_obj(point, orien, obj_size, obj_name, obj_color=[1.0, 0.0, 0.0]):
        scene2= PlanningSceneInterface("base_link")
        obj_pose = {}
        obj_pose['position']=point
        obj_pose['orientation']=orien  #xyzw
        robot_control.create_object(obj_size, obj_pose, obj_name)
        [obj_r, obj_g, obj_b] = obj_color
        scene2.setColor(obj_name, obj_r, obj_g, obj_b)
        scene2.sendColors()

    
    @staticmethod
    def world_pose(image, aruco_dict, ar_size=0.05):
        if image is None:
            print('video caputre failed')
        print('start estimate pose')
        arucopose.PoseEstimate(image, aruco_dict, aruco_size=ar_size)
        end_position, end_orientation = robot_control.info_get()
        translation = arucopose.to_wordcoord(end_position, end_orientation)
        # print('translation:', translation)
        return translation
        
    def table_mapping(self):
        # global color_image
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        #===== control robot reach front point
        robot_control.set_joints_values(joints_front)
        print('Arrived')
        time.sleep(3)
        imag = rs_image.get_image()
        
        tr_f = ArucoMapping.world_pose(imag, aruco_dict)
        print(tr_f)

        #===== split the process of control robot move to the back point
        robot_control.set_joints_values(joints_back)
        print('Arrived')
        time.sleep(3)
      
        
        imag = rs_image.get_image()
        # print('imag')
        # time.sleep(3)
        tr_b = ArucoMapping.world_pose(imag, aruco_dict)
        
        
        # print('tr_b:', tr_b)
        end_position, _ = robot_control.info_get()
        labels = [50, 140]
        pfl, pfr, pbr, pbl = self.cuboid_geometry(tr_f, tr_b, name='table', label=labels)
        print('point1:', pfl, '\npoint2:', pfr, '\npoint3:', pbr)
        print('point4', pbl)
        
        # plot scatter points
        X = [pfl[0], pfr[0], pbr[0], pbl[0], end_position[0]]
        Y = [pfl[1], pfr[1], pbr[1], pbl[1], end_position[1]]
        Z = [pfl[2], pfr[2], pbr[2], pbl[2], end_position[2]]
            
        # Plot the scatter points
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the scatter points
        ax.scatter(X[:-1], Y[:-1], Z[:-1], color='red')  # Plot all points except the last one
        ax.scatter(X[-1], Y[-1], Z[-1], color='blue')    # Plot the last point with a different color
        # Add labels and title to the plot
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Scatter Plot')

        # Save the plot as an image file
        plt.savefig(collision_path +'corner of table.png')
        # Display the plot
        plt.show()
    
    def base_mid_mapping(self, area, others=None):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        aruco_square = 0.015

        imag = rs_image.get_image()
        
        
        if area =='mid':
            robot_control.set_joints_values(base_mid)
            tr_all = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)
            tr_f = {key: tr_all[key] for key in [20, 2]}
            tr_b = {key: tr_all[key] for key in [3, 19]}
            labels = [2, 19]
            print('tr_f', tr_f, '\ntr_b', tr_b)
            self.base_geometry(tr_f, tr_b, 'base_mid2', label=labels, refer_qua =tr_all[3][1], direction= 'vertical')
            
            # translation = ArucoMapping.world_pose(imag, aruco_dict, ar_size=0.022)
            # cuboid_size = [0.23, 0.3959780869066051, 0.02]
            # id = 2
            # self.singal_aruco_object(translation, id, cuboid_size, 'area', 'special')
            # obj_pose = {}
            # obj_pose['position']=translation[id][0]
            # obj_pose['orientation']=translation[id][1]  #xyzw
            # cuboid_size = [0.23, 0.3959780869066051, 0.02]
            # robot_control.create_object(cuboid_size, obj_pose, 'mid_back')
            
            cuboid_size = [0.16, 0.16, 0.04]
            ori_pose={}
            ori_pose['position'] = [0, 0, -0.023]
            ori_pose['orientation'] = [0, 0, 0, 1]
            robot_control.create_object(cuboid_size, ori_pose, 'original')
            
        elif area =='base':
            robot_control.set_joints_values(base_bottom_front)
            time.sleep(3)
            imag = rs_image.get_image()
            tr_f_all = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)
            tr_f = {key: tr_f_all[key] for key in [4, 5]}
            robot_control.set_joints_values(base_bottom_back)
            time.sleep(3)
            imag = rs_image.get_image()
            tr_b_all = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)
            tr_b = {key: tr_b_all[key] for key in [0, 1]}
            labels = [4, 1]
            self.base_geometry(tr_f, tr_b, 'base_bottom', label=labels, refer_qua =tr_b[1][1], direction= 'horizontal')
            
        elif area == 'mid_back':
            robot_control.set_joints_values(base_mid)
            imag = rs_image.get_image()
            translation = ArucoMapping.world_pose(imag, aruco_dict, ar_size=0.022)
            cuboid_size = [0.3959780869066051, 0.23, 0.02]
            id = 18
            self.singal_aruco_object(translation, id, cuboid_size, 'area', 'special')


        
        elif area == 'motor':
            joints_right = [2.7054593563079834, -2.3169167677508753, -1.6796091238604944, -0.4096568266498011, -0.6313355604754847, 0.27382418513298035]
            robot_control.set_joints_values(joints_right)
            time.sleep(3)
            imag = rs_image.get_image()
            tr_f_all = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)
            tr_f = {key: tr_f_all[key] for key in [17, 6]}
            
            joints_left = [2.705507278442383, -2.31688100496401, -1.6796334425555628, -0.7866719404803675, -0.1286247412310999, -1.7368601004229944]
            robot_control.set_joints_values(joints_left)
            time.sleep(3)
            imag = rs_image.get_image()
            tr_b_all = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)
            tr_b = {key: tr_b_all[key] for key in [11, 12]}

            labels = [6, 11]
            pfl, pfr, pbr, pbl = self.cuboid_geometry(tr_f, tr_b, name='motor', label=labels)
            
            
            
            
        if others:
            #base_left joints: [2.831394910812378, -2.821069065724508, -1.4930074850665491, 0.46678268909454346, -0.8852956930743616, -5.2779555956469935]
            # peak:joints: [2.831382989883423, -2.8210929075824183, -1.4156468550311487, 1.5150138139724731, -1.2703107039081019, -5.529240850602285]
    
            imag = rs_image.get_image()
            translation = ArucoMapping.world_pose(imag, aruco_dict, ar_size=0.022)
            # id = 0
            # id = 2 #base_left
            id = 1 # peak 1
            id2 = 11
            # cuboid_size = [0.3959780869066051, 0.20, 0.2]
            # cuboid_size = [0.04,  0.40, 0.21] # base_left
            # cuboid_size = [0.59,  0.046, 0.15]
            # cuboid_size = [0.04, 0.58,  0.15]
            
            # self.singal_aruco_object(translation, id, cuboid_size, 'peak1', direction='vertical')
            
            cuboid_size = [0.04, 0.26,  0.15]
            
            self.singal_aruco_object(translation, id2, cuboid_size, 'peak2', direction='vertical')
            # self.singal_aruco_object(translation, id2, cuboid_size, 'peak2', direction='horizontal')
        
    
    def single_collision_mapping(self):
        # global color_image
        # filename = "object_data.csv"
        
        # filepath = os.path.join(collision_path, filename)

        aruco_square = 0.015
        # aruco_square = 0.05

        aruco_color = [0.0, 0.0, 1.0]
        # column_name = ['name', 'width', 'length','height', 'center x', 'center y', 'center z']
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        time.sleep(2)
        
        imag = rs_image.get_image()
        # imag = cv2.imread('1.jpg')
        # imag = np.array(imag)
        aruco_pose = ArucoMapping.world_pose(imag, aruco_dict, ar_size=aruco_square)

        csv_row = {}
        obj_size = [aruco_square, aruco_square, aruco_square]
        for id in aruco_pose.keys():
            # id = ids[i]
            point = aruco_pose[id][0]
            orien = aruco_pose[id][1]
            print('id 0:', id)
            row_name = 'collision' + str(id) 
            point[2] += aruco_square/2                  # only suitable for aruco markers are on the surface; this should be point[2] -= aruco_square/2 for aruco are attached on objects
            csv_row[row_name] = [row_name, aruco_square, aruco_square, aruco_square, point[0], point[1], point[2]]

            print('point:', point, '\norien:', orien)
            # #===== add collision object====
            ArucoMapping.collision_obj(point, orien, obj_size, row_name, obj_color=aruco_color)
            
        ArucoMapping.csv_save(csv_row)
    

joints_front=  [2.7046329975128174, -2.8212369124041956, -0.7389286200152796, -1.2893827597247522, -1.7621658484088343, 0.3994840383529663]
#     joints_left = [2.7055788040161133, -2.8210209051715296, -1.3673651854144495, -1.7919729391681116, -1.0078628698932093, 1.0271031856536865]

joints_back = [2.7046449184417725, -2.821451489125387, -1.2412942091571253, -1.9179843107806605, -0.8818882147418421, 1.5303915739059448]


base_mid = [2.7055552005767822, -2.820817772542135, -1.0532053152667444, 0.846084713935852, -1.1330707708941858, 2.4096720218658447]


base_bottom_front = [2.705674886703491, -2.8220985571490687, -1.7432869116412562, 0.7185221910476685, -0.8844803015338343, 1.0254268646240234]

base_bottom_back =  [2.7060463428497314, -2.821679417287008, -1.99518329301943, 0.8436025381088257, -1.011197868977682, -3.4977334181415003]

real_camera_img_aligned = None

if __name__ == "__main__":
    collision_path = './Collision_object/'

    robot_control = Ur5_env()
    arucopose = aruco_pose()
    code_mapping = ArucoMapping()
    rs_image = ros_camera()
    
    # code_mapping.table_mapping()                  # add table as collision object
    code_mapping.base_mid_mapping('mid')
    # code_mapping.base_mid_mapping('base')
    # code_mapping.base_mid_mapping('motor')
    # code_mapping.base_mid_mapping('mid_back')
    # code_mapping.base_mid_mapping('1', others='base_right')
    # code_mapping.base_mid_mapping('1', others='base_left')
    
    # code_mapping.single_collision_mapping()         # add aruco markers as collision object
