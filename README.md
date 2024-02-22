# single_ur5_with_moveit
## Usage
```
#====Start manipulator====
source devel_isolated/setup.bash
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.8.100    #right ur5
roslaunch ur5_moveit_config moveit_planning_execution.launch
roslaunch ur_control moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
roslaunch realsense2_camera rs_camera.launch

#===charuco calibration===
python realsense_img.py
python ChArUco.py
```
## Collision objects
In moveit_rviz.launch:
```
  <node pkg="ur_control" type="add_frame.py" name="broadcaster_fixed" />
  <node pkg="ur_control" type="tf_listener.py" name="frame_listener" />
  
  <arg name="scene_file" default="$(find ur_control)/launch/collision_version3.scene"/> 
  <node name = "moveit_publish_scene_from_text" pkg= "moveit_ros_planning" type = "moveit_publish_scene_from_text" args= "$(arg scene_file)"/>
```
the two *.py files change the orientation of ur manipulator base frame and make the ur5 matches rotation in real hardware.

By loading .scene file, we add the collison objects information in reality like table.


### collision_version3.scene:
```
(noname)+
* base_bottom
-0.0168411 -0.532449 0.223739
0.655474 0.647393 0.26583 -0.28385
1
box
0.444658 0.246102 0.01
0 0 0
0 0 0 1
0 0 1 0.9
0
...
```

The poses of collision objects are relative to the reference frame (position = [0, 0, 0], orientation = [0, 0, 0, 1]). Transformation may be necessary if the world reference frame is changed or if there is a translation between the base frame and the world reference frame.
