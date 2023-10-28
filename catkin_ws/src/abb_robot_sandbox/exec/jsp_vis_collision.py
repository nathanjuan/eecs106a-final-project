#!/usr/bin/env python
import os
import rospy
from abb_robot_sandbox.vis import Vis
from abb_robot_sandbox.col import CollisionBody, ColHandler
from abb_robot_sandbox.util import TransformManager, reorder_joints, RobotKinematics
from rospkg import RosPack
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as ParallelGripperInputMsg


class Jsp_Vis_Col:
    def __init__(self):
        self.robot_config = 7 * [0.]
        self.finger_joint_pos = [0.]
        self.link_names = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "flange_interface"]
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "finger_joint"]
        self.vis = Vis("base_link", "wrist_3_link", self.link_names)
        rp = RosPack()
        self.tm = TransformManager()
        self.robot_cb = CollisionBody()
        self.open_gripper_cb = CollisionBody()
        self.close_gripper_cb = CollisionBody()
        self.obstacle_cb = CollisionBody()
        path_to_package = rp.get_path('abb_robot_sandbox')
        self.robot_cb.load(os.path.join(path_to_package, 'config/ur5_collision_spheres.yaml'))
        self.obstacle_cb.load(os.path.join(path_to_package, 'config/box_collision_spheres.yaml'))
        self.blue = [0, 0, 1, 1]
        self.yellow = [1, 1, 0, 1]
        self.col = ColHandler(self.robot_cb, self.joint_names)
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback, queue_size=1)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            for i, link_name in enumerate(self.link_names):
                for j, sphere in enumerate(self.robot_cb.get_spheres(link_name)):
                    sphere_base_link_pos = self.tm.transform_point(
                        sphere[:3], self.joint_names, self.robot_config, link_name, "base_link")
                    self.vis.visualize_sphere(
                        sphere_base_link_pos,
                        sphere[3],
                        rgba=self.yellow if sphere[4] == 1 else self.blue,
                        marker_ns="col_vis_" + link_name,
                        marker_id=j)

            for j, sphere in enumerate(self.robot_cb.get_spheres("base_link")):
                self.vis.visualize_sphere(
                    sphere[:3],
                    sphere[3],
                    rgba=self.blue,
                    marker_ns="col_vis_base_link",
                    marker_id=j)
            self.vis.visualize_cube(
                    [0.0478, -0.4657, 0.2342],
                    [0.3242, 0.1704, 0.4683],
                    rgba=self.yellow,
                    marker_ns="box_vis",
                    marker_id=0)

    def joint_states_callback(self, message):
        if len(message.position) == 6:
            given_config = list(message.position) + [0]
            given_joint_names = list(message.name) + ["finger_joint"]
            self.robot_config = reorder_joints(
                given_config, given_joint_names, self.joint_names)



def main():
    rospy.init_node('jsp_vis_collision')
    jsp_vis_col = Jsp_Vis_Col()
    jsp_vis_col.run()


if __name__ == '__main__':
    main()
