# eecs106a-final-project
Final Project for EECS 106A Fall 2023.
Rishi Yang
Yarden Goraly
Nathan Luan

Vint Lee

UR5 Guide: https://docs.google.com/document/d/1LiBr485hCvnpTVi90FoljeGexzEYnljqE2dXsjd6uik/edit#heading=h.kc2ws2ma7twt
Robotiq Source Repo: https://github.com/TAMS-Group/robotiq

TODO

make script that can
    find a block (find end effector position relative to ar tag? and block location relative to ar tag?)
        *precursor: add gripper to arm model, to build better trajectories
        callibrate blue HSV value
        debug scaling on depth units of realsense camera
        one idea -- find location of ar tag using depth camera and ar function, compare results?
    move robot arm to block
        need to download UR_driver and moveit_config and ros noetic
        figure out trajectory planner
    pick up block with robotiq hand
        we mostly figured this out

roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=172.22.22.2

or roslaunch ur_gazebo ur5_bringup.launch

roslaunch ur5_moveit_config moveit_planning_execution.launch (sim:=true)

*ROSLAUNCH for gripper**

roslaunch ur5_moveit_config moveit_rviz.launch

roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true depth_fps:=6 color_fps:=6

roslaunch lab4_cam ar_track.launch

rosrun movement ar_to_base_transform.py
