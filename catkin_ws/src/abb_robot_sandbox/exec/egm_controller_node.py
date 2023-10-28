#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState
import abb_robot_sandbox
from abb_robot_sandbox.msg import ControlGoal, ControlFeedback, ControlResult
import actionlib
from abb_robot_sandbox.util import RobotKinematics
from visualization_msgs.msg import Marker
from abb_robot_msgs.srv import TriggerWithResultCode
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from abb_rapid_sm_addin_msgs.srv import GetEGMSettings, SetEGMSettings
from abb_egm_msgs.msg import EGMState
import math


class EGMController:
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

        # RWS services.
        self._stop_rapid_srv = rospy.ServiceProxy(
            '/rws/stop_rapid', TriggerWithResultCode)
        self._pp_to_main_srv = rospy.ServiceProxy(
            '/rws/pp_to_main', TriggerWithResultCode)
        self._start_rapid_srv = rospy.ServiceProxy(
            '/rws/start_rapid', TriggerWithResultCode)
        self._start_egm_joint_srv = rospy.ServiceProxy(
            '/rws/sm_addin/start_egm_joint', TriggerWithResultCode)
        self._stop_egm_srv = rospy.ServiceProxy(
            '/rws/sm_addin/stop_egm', TriggerWithResultCode)

        self._switch_controller_srv = rospy.ServiceProxy(
            '/egm/controller_manager/switch_controller', SwitchController)

        self._vel_cmd_pub = rospy.Publisher(
            '/egm/joint_group_velocity_controller/command',
            Float64MultiArray, queue_size=10)
        self._pos_cmd_pub = rospy.Publisher(
            '/egm/joint_group_position_controller/command',
            Float64MultiArray, queue_size=10)

        self._get_egm_settings_srv = rospy.ServiceProxy(
            "/rws/sm_addin/get_egm_settings", GetEGMSettings)
        self._set_egm_settings_srv = rospy.ServiceProxy(
            "/rws/sm_addin/set_egm_settings", SetEGMSettings)

        self._egm_state_sub = rospy.Subscriber(
            '/egm/egm_states', EGMState, self._egm_state_callback)
        self._egm_joint_state_sub = rospy.Subscriber(
            '/egm/joint_states', JointState, self._egm_joint_state_callback)
        self._egm_state = None
        self._egm_joint_state = None

        # For visualizing the desired EE position.
        if not rospy.has_param('~robot_base_frame_id'):
            return False
        self._robot_base_frame_id = rospy.get_param(
            '~robot_base_frame_id')

        if not rospy.has_param('~robot_tip_frame_id'):
            return False
        self._robot_tip_frame_id = rospy.get_param(
            '~robot_tip_frame_id')

        # Initialize the robot kinematics (FK/IK).
        self._kinematics = RobotKinematics(
            self._robot_base_frame_id, self._robot_tip_frame_id)

        if not rospy.has_param('~topics/vis'):
            return False
        vis_topic = rospy.get_param('~topics/vis')
        self._vis_pub = rospy.Publisher(vis_topic, Marker, queue_size=25)

        if not rospy.has_param('~joint_pos_tol'):
            return False
        self._joint_pos_tol = rospy.get_param('~joint_pos_tol')

        return True

    def _egm_state_callback(self, msg):
        self._egm_state = msg

    def _egm_joint_state_callback(self, msg):
        self._egm_joint_state = msg

    def start_egm_session(self):
        current_settings = self._get_egm_settings_srv(task='T_ROB1')
        settings = current_settings.settings
        print(settings)

        settings.activate.max_speed_deviation = math.degrees(3.0)
        print(settings)
        self._set_egm_settings_srv(task='T_ROB1', settings=settings)

        rospy.loginfo("[Controller] Stopping RAPID...")
        res = self._stop_rapid_srv()
        if res.result_code != 1:
            rospy.logerr("[Controller] {}".format(res.message))
            # return False

        rospy.sleep(2)

        rospy.loginfo("[Controller] Setting program pointer to main...")
        res = self._pp_to_main_srv()
        if res.result_code != 1:
            rospy.logerr("[Controller] {}".format(res.message))
            # return False

        rospy.sleep(2)

        rospy.loginfo("[Controller] Starting RAPID...")
        res = self._start_rapid_srv()
        if res.result_code != 1:
            rospy.logerr("[Controller] {}".format(res.message))
            # return False

        rospy.sleep(2)

        rospy.loginfo("[Controller] Starting EGM session...")
        res = self._start_egm_joint_srv()
        if res.result_code != 1:
            rospy.logerr("[Controller] {}".format(res.message))
            return False

        rospy.sleep(0.5)

        rospy.loginfo("[Controller] ...done!")
        return True

    def stop_egm_session(self):
        rospy.loginfo("[Controller] Stopping EGM...")
        res = self._stop_egm_srv()
        if res.result_code != 1:
            rospy.logerr("[Controller] {}".format(res.message))
            return False

        rospy.loginfo("[Controller] ...done")
        return True

    def start_velocity_controller(self):
        rospy.loginfo(
            "[Controller] Switching to joint group velocity controller...")
        res = self._switch_controller_srv(
            ['joint_group_velocity_controller'], [], 1, False, 0.0)
        if not res.ok:
            return False

        rospy.loginfo("[Controller] ...done")
        return True

    def start_position_controller(self):
        rospy.loginfo(
            "[Controller] Switching to joint group position controller...")
        res = self._switch_controller_srv(
            ['joint_group_position_controller'], [], 1, False, 0.0)
        if not res.ok:
            return False

        rospy.loginfo("[Controller] ...done")
        return True

    def callback(self, goal):
        rospy.loginfo("[Controller] Received trajectory with {} waypoints".
                      format(len(goal.trajectory.points)))

        rate = rospy.Rate(self._freq)

        feedback = ControlFeedback()
        time = rospy.Time.now()
        success = True

        self._egm_state = None
        self._egm_joint_state = None

        index = 0

        if not self.start_egm_session():
            result = ControlResult()
            result.result = ControlResult.FAILURE
            self._action_server.set_succeeded(result)
            return

        # if not self.start_velocity_controller():
        if not self.start_position_controller():
            result = ControlResult()
            result.result = ControlResult.FAILURE
            self._action_server.set_succeeded(result)
            return

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                rospy.logwarn("[Controller] Preempt requested!")
                self._action_server_.set_preempted()
                success = False
                break

            if self._egm_state is None:
                rospy.logwarn("[Controller] Waiting for EGM state message...")
                rate.sleep()
                continue
            else:
                if len(self._egm_state.egm_channels) == 0 or \
                   not self._egm_state.egm_channels[0].active:
                    rospy.logwarn("[Controller] EGM is not active!")
                    success = False
                    break

            if self._egm_joint_state is None:
                rospy.logwarn(
                    "[Controller] Waiting for EGM joint state message...")
                rate.sleep()
                continue

            advance_index = True
            for i in range(6):
                if abs(goal.trajectory.points[index].positions[i] -
                       self._egm_joint_state.position[i]) > self._joint_pos_tol:
                    advance_index = False
                    break

            if advance_index:
                index += 1

            if index >= len(goal.trajectory.points):
                rospy.loginfo("[Controller] Reached end of trajectory!")
                break

            # time_from_start = rospy.Time.now() - time

            # if goal.trajectory.points[-1].time_from_start <= time_from_start:
            #     rospy.loginfo("[Controller] Reached end of trajectory!")
            #     break

            # index = -1
            # for i in range(len(goal.trajectory.points) - 1):
            #     point = goal.trajectory.points[i]
            #     next_point = goal.trajectory.points[i + 1]
            #     if point.time_from_start < time_from_start and \
            #        time_from_start <= next_point.time_from_start:
            #         index = i
            #         break

            rospy.loginfo("[Controller] At index {}".format(index))
            if index < 0:
                success = False
                break

            # Feedback gives the current desired joint config.
            joint_states_msg = JointState()
            joint_states_msg.position = goal.trajectory.points[index].positions
            # self._joint_states_pub.publish(joint_states_msg)

            # Publish the joint position commands to EGM.
            pos_cmd_msg = Float64MultiArray()
            pos_cmd_msg.data = goal.trajectory.points[index].positions
            self._pos_cmd_pub.publish(pos_cmd_msg)

            self.pub_ee_marker(pos_cmd_msg.data)

            feedback.config = joint_states_msg
            self._action_server.publish_feedback(feedback)

            rate.sleep()

        result = ControlResult()
        if success:
            result.result = ControlResult.SUCCESS
        else:
            result.result = ControlResult.FAILURE

        self.stop_egm_session()

        self._action_server.set_succeeded(result)

    def pub_ee_marker(self, config):
        msg = Marker()
        msg.header.frame_id = self._robot_base_frame_id
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.ns = 'controller/ee'
        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0
        msg.color.a = 0.75

        pos, _ = self._kinematics.fk(config)
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        self._vis_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    rospy.init_node('egm_controller')

    controller = EGMController(rospy.get_name())
    if not controller.init():
        rospy.logerr("[EGMController] Error: failed to initialize!")
        sys.exit(-1)

    controller.run()


if __name__ == '__main__':
    main()
