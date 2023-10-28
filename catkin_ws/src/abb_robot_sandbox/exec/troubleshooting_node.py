#!/usr/bin/env python
import rospy
from abb_robot_sandbox.rrt_v2 import RRT
from rospkg import RosPack
from abb_robot_sandbox.col import CollisionBody, ConfigurationValidator
from abb_robot_sandbox.util import RobotKinematics
from rrt_motion_planner_node import RRTMotionPlanner
import os
import random
import numpy
import math

rospy.init_node("rrt_troubleshooting")

kin = RobotKinematics("wrist_3_link")
rp = RosPack()
path_to_package = rp.get_path('abb_robot_sandbox')
robot_cb = CollisionBody()
robot_cb.load(os.path.join(path_to_package, 'config/ur5_collision_spheres.yaml'))
obstacle_cb = CollisionBody()
obstacle_cb.load(os.path.join(path_to_package, 'config/box_collision_spheres.yaml'))
joint_count = 6
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
link_names = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
config_validator = ConfigurationValidator(robot_cb, joint_names, [obstacle_cb], "finger_joint")
config_validator.set_finger_joint_poss([0])
print(kin.fk(joint_names, [-4.027069632207052, -1.894285027180807, -1.43235952058901, -1.4164159933673304, 1.5189874172210693, 1.9736438989639282]))
print(kin.ik_valid(config_validator, [0.327239074573171, -0.49243656139301273, 0.3575251351900703], [0.4191502091213263, -0.5559395727120471, 0.5873937775040035, 0.4125685928859253]))
