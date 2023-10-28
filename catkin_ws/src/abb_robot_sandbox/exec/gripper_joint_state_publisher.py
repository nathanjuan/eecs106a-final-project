#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg._Robotiq2FGripper_robot_input \
    import Robotiq2FGripper_robot_input
import numpy as np


class GripperJointStatePublisher:
    def __init__(self):
        pass

    def init(self):
        self._freq = rospy.get_param('~freq', 30)
        self._finger_joint_position = rospy.get_param(
            '~finger_joint_position', 0)
        self._finger_joint_name = rospy.get_param(
            '~finger_joint_name', 'finger_joint')

        joint_states_topic = rospy.get_param(
            '~joint_states_topic', 'joint_states')
        self._joint_states_pub = rospy.Publisher(
            joint_states_topic, JointState, queue_size=10)

        robotiq_gripper_status_topic = rospy.get_param(
            '~robotiq_gripper_status_topic', 'Robotiq2FGripperRobotInput')
        self._robotiq_gripper_status_sub = rospy.Subscriber(
            robotiq_gripper_status_topic,
            Robotiq2FGripper_robot_input,
            self.robotiq_gripper_status_callback)

        self._gripper_stroke = rospy.get_param('~gripper_stroke', 0.140)
        self._gripper_joint_limit = rospy.get_param('~gripper_joint_limit', 0.7)

        return True

    def robotiq_gripper_status_callback(self, msg):
        # Get the current position of the gripper.
        pos = np.clip(self._gripper_stroke / (3 - 230) * (
            float(msg.gPO) - 230), 0, self._gripper_stroke)
        self._finger_joint_position = np.clip(
            self._gripper_joint_limit - ((self._gripper_joint_limit /
                                          self._gripper_stroke) * pos),
            0,
            self._gripper_joint_limit)

    def run(self):
        rate = rospy.Rate(self._freq)

        msg = JointState()
        msg.name = [self._finger_joint_name]

        msg.velocity = [0]
        msg.effort = [0]

        while not rospy.is_shutdown():
            msg.position = [self._finger_joint_position]
            msg.header.stamp = rospy.Time.now()
            self._joint_states_pub.publish(msg)

            rate.sleep()


def main():
    rospy.init_node('gripper_joint_state_publisher')

    pub = GripperJointStatePublisher()
    if not pub.init():
        rospy.logerr("Failed to initialize!")
        sys.exit(-1)

    pub.run()


if __name__ == '__main__':
    main()


