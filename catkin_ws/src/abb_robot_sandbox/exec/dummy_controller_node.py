#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState
import abb_robot_sandbox
from abb_robot_sandbox.msg import ControlGoal, ControlFeedback, ControlResult
import actionlib


class DummyController:
    def __init__(self, name):
        self._action_server = actionlib.SimpleActionServer(
            name,
            abb_robot_sandbox.msg.ControlAction,
            execute_cb=self.callback,
            auto_start=False)
        self._action_server.start()

    def init(self):
        if not rospy.has_param('~freq'):
            return False
        self._freq = rospy.get_param('~freq')

        if not rospy.has_param('~topics/joint_states_in'):
            return False
        self._joint_states_pub = rospy.Publisher(
            rospy.get_param('~topics/joint_states_in'), JointState, queue_size=15)

        return True

    def callback(self, goal):
        rospy.loginfo("[Controller] Received trajectory with {} waypoints".
                      format(len(goal.trajectory.points)))

        rate = rospy.Rate(self._freq)

        feedback = ControlFeedback()
        time = rospy.Time.now()
        success = True
        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                rospy.logwarn("[Controller] Preempt requested!")
                self._action_server_.set_preempted()
                success = False
                break

            time_from_start = rospy.Time.now() - time

            if goal.trajectory.points[-1].time_from_start <= time_from_start:
                rospy.loginfo("[Controller] Reached end of trajectory!")
                break

            index = -1
            for i in range(len(goal.trajectory.points) - 1):
                point = goal.trajectory.points[i]
                next_point = goal.trajectory.points[i + 1]
                if point.time_from_start < time_from_start and \
                   time_from_start <= next_point.time_from_start:
                    index = i
                    break

            rospy.loginfo("[Controller] At index {}".format(index))
            if index < 0:
                success = False
                break

            # TODO Note: here we just send the exact joint positions.
            joint_states_msg = JointState()
            joint_states_msg.position = goal.trajectory.points[index].positions
            self._joint_states_pub.publish(joint_states_msg)

            feedback.config = joint_states_msg
            self._action_server.publish_feedback(feedback)

            rate.sleep()

        result = ControlResult()
        if success:
            result.result = ControlResult.SUCCESS
        else:
            result.result = ControlResult.FAILURE

        self._action_server.set_succeeded(result)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('dummy_controller')

    controller = DummyController(rospy.get_name())
    if not controller.init():
        rospy.logerr("[DummyController] Error: failed to initialize!")
        sys.exit(-1)

    controller.run()


if __name__ == '__main__':
    main()
