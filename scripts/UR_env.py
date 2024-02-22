#!/usr/bin/env python
import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from copy import deepcopy
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
import shape_msgs.msg
# from robot_msgs.msg import RobotMsg
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class Ur5_env:
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('listener',anonymous=True)
    reference_frame = "base_link"
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

    end_effector_link = move_group.get_end_effector_link()
    
    # move_group.set_pose_reference_frame(reference_frame)
        
    # allow replanning after failed
    move_group.allow_replanning(True)

    # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_goal_joint_tolerance(0.001)

    
    # 设置允许的最大速度和加速度
    move_group.set_max_acceleration_scaling_factor(0.5)
    move_group.set_max_velocity_scaling_factor(0.2)
    
    # robot_state_msg = RobotMsg()
    fixed_frame = "dummy"

    def info_get(self):
         # get pose of end effectror in planning frame
        end_effector_link = self.move_group.get_end_effector_link()
        # reference_frame = self.fixed_frame
        # self.move_group.set_pose_reference_frame(reference_frame)
        ref = self.move_group.get_pose_reference_frame()
        # print('ref', ref)
        # plan_frame = self.move_group.get_planning_frame()
        # print('plan_frame', plan_frame)
        current_pose = self.move_group.get_current_pose(end_effector_link).pose
        position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        
        return position, orientation
    
    def endpose_world(self):
        reference_world = self.fixed_frame
        current_pose = self.move_group.get_current_pose(reference_world).pose
        print('dummy:', current_pose)
        base_link = self.reference_frame
        current_pose = self.move_group.get_current_pose(base_link).pose
        print('base_link', current_pose)
        
        
    
    def get_joints(self):
        
        joints_val =  self.move_group.get_current_joint_values()
        
        return joints_val
        
 
    def set_joints_values(self, joints_values):
        self.move_group.set_joint_value_target(joints_values)

        # plan1 = self.move_group.plan(joints_values)
        # time.sleep(2)
        # Unpack the tuple
        plan_success, trajectory, planning_time, _ = self.move_group.plan(joints_values)
        time.sleep(2)
        if plan_success:
            print('safe')
            # print('plan1', plan1)
            self.move_group.go(wait=True)
            time.sleep(5)
        else:
            print('Motion planing failed')
            
        # plan = self.move_group.plan()

        # if not plan.joint_trajectory.points:
        #     print('failed')
        # self.move_group.go()
        
        
        # self.state_feedback(joints_values, 'joint')
        
        # tolerance = self.move_group.get_goal_joint_tolerance()
        # break_outer_loop = False
        # while True:
        #     state_msg = self.robot_state_msg.state
        #     print('state_msg', state_msg)
        #     if state_msg == 0:
        #         # break
        #         current_joint_positions = self.move_group.get_current_joint_values()

        #         for current_joint, expected_joint in zip(current_joint_positions, joints_values):
        #             joint_error = abs(current_joint - expected_joint)
        #             if joint_error > tolerance:
        #                 # print(f"Joint deviation exceeds tolerance: {joint_error}")
        #                 self.move_group.set_joint_value_target(joints_values)
        #                 self.move_group.go()
        #                 time.sleep(3)
        #             else:
        #                 break_outer_loop = True
        #                 break
        #         if break_outer_loop:
        #             break

    def create_object(self, obj_size, obj_pose, name):
        # We can get the name of the reference frame for this robot:
        frame_id = self.move_group.get_planning_frame()
        print('frame_id', frame_id)
        # frame_id = "dummy"
        collision_object = moveit_msgs.msg.CollisionObject()
        collision_object.header.frame_id = frame_id
        # name = "box1"
        collision_object.id = name
        primitive = shape_msgs.msg.SolidPrimitive()
        # Define the size of the box in meters
        primitive.type = primitive.BOX
        # table_size = [0.1, 0.1, 0.01]
        primitive.dimensions = obj_size
        
        posiziton = obj_pose['position']
        orien = obj_pose['orientation']
        box_pose = geometry_msgs.msg.Pose()
        pos = box_pose.position
        ori = box_pose.orientation
        pos.x, pos.y, pos.z  = posiziton
        ori.x, ori.y, ori.z, ori.w = orien

        # print('frame_id:', frame_id, 'collision_object.id :', collision_object.id)
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = collision_object.ADD 

        self.scene.add_object(collision_object)
        rospy.sleep(2)  
        # Set the start state to the current state of the robot
        self.move_group.set_start_state(self.robot.get_current_state())

        
joints_front=  [2.7055673599243164, -2.8210328261004847, -1.2416423002826136, -1.1637118498431605, -1.6360052267657679, 1.0270792245864868]
# joints_right =

#     joints_left = [2.7055788040161133, -2.8210209051715296, -1.3673651854144495, -1.7919729391681116, -1.0078628698932093, 1.0271031856536865]

joints_back = [2.7055788040161133, -2.8210209051715296, -1.3673651854144495, -1.7919729391681116, -1.0078628698932093, 1.0271031856536865]

base_mid =[2.7064058780670166, -2.8205540815936487, -1.0533011595355433, 0.846624493598938, -1.0068066755877894, 0.9009111523628235]
base_bottom_front = [2.706022262573242, -2.8217151800738733, -1.743946377431051, 0.8437463045120239, -0.8853681723224085, -0.23060495058168584]
base_bottom_back =  [2.7060463428497314, -2.821679417287008, -1.99518329301943, 0.8436025381088257, -1.011197868977682, -3.4977334181415003]

if __name__ == "__main__":
    robot_env = Ur5_env()
    joints_right =[2.7055673599243164, -2.8210328261004847, -1.2416423002826136, -1.1637118498431605, -1.6360052267657679, 1.0270792245864868]

    joints_left = [2.7055788040161133, -2.8210209051715296, -1.3673651854144495, -1.7919729391681116, -1.0078628698932093, 1.0271031856536865]
    position, orientation = robot_env.info_get()
    print('position', position)
    print('orientation', orientation)
    
    # robot_env.endpose_world()
    joint_values = robot_env.get_joints()
    print('joints:', joint_values)
    
    # joints_pre =  [2.7060463428497314, -2.8214996496783655, -0.9282215277301233, -1.5401213804828089, -1.3843124548541468, 0.9005160331726074]

    # robot_env.set_joints_values(base_bottom_back)