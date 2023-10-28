#!/usr/bin/env python

import rospy
import sys
import actionlib
import abb_robot_sandbox
from abb_robot_sandbox.msg import PlanGoal, PlanFeedback, PlanResult
from abb_robot_sandbox.util import RobotKinematics
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
import PyKDL
from angles import shortest_angular_distance, normalize_angle
import tf.transformations as trans
import numpy as np


class KinematicsHelper:
    def __init__(self, chain):
        self._chain = chain

    def skew_symmetric(self, v):
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

    def adjoint(self, trans):
        adj = np.zeros((6, 6))

        rot = trans[0:3, 0:3]
        pos = trans[0:3, 3]

        adj[0:3, 0:3] = rot
        adj[0:3, 3:] = self.skew_symmetric(pos).dot(rot)
        adj[3:, 3:] = rot

        return adj

    def axis_angle_to_rot(self, axis, angle):
        w = self.skew_symmetric(axis)
        return np.eye(3) + np.sin(angle) * w + (1 - np.cos(angle)) * w.dot(w)

    def screw_to_trans(self, screw, theta):
        trans = np.eye(4)

        trans_axis = screw[0:3]
        rot_axis = screw[3:]

        trans[0:3, 0:3] = self.axis_angle_to_rot(rot_axis, theta)

        w = self.skew_symmetric(rot_axis)
        trans[0:3, 3:] = (
            np.eye(3) * theta + (1 - np.cos(theta)) *
            w + (theta - np.sin(theta)) * w.dot(w)).dot(
                trans_axis)

        return trans

    def shortest_angular_dist(self, first_orien, second_orien):
        p = np.array([
            first_orien[0], first_orien[1],
            first_orien[2], first_orien[3]])
        q = np.array([
            second_orien[0], second_orien[1],
            second_orien[2], second_orien[3]])
        return np.fmod(2 * np.arccos(p.dot(q)), 2 * np.pi)

    def jac(self, pos):
        jac = np.zeros((6, 6))

        trans = []
        base_to_root = np.eye(4)

        for i in range(6):
            segment = self._chain.getSegment(i)
            joint = segment.getJoint()
            angle = pos[i]

            joint_origin = np.array([[joint.JointOrigin().x()],
                                     [joint.JointOrigin().y()],
                                     [joint.JointOrigin().z()],
                                     [1.0]])

            q = (base_to_root.dot(joint_origin))[0:3, 0:]
            omega = np.array([[joint.JointAxis().x()],
                              [joint.JointAxis().y()],
                              [joint.JointAxis().z()]])

            twist = np.zeros((6, 1))
            twist[0:3, 0:] = -np.cross(omega, q, axis=0)
            twist[3:, 0:] = omega

            if i == 0:
                # Construct the first column.
                jac[0:, 0:1] = twist
            else:
                t = np.eye(4)
                for j in range(len(trans)):
                    t = t.dot(trans[j])

                adj = self.adjoint(t)

                jac[0:, i:i+1] = adj.dot(twist)

            trans.append(self.screw_to_trans(twist, angle))

            root_to_tip = np.eye(4)

            # Rotation.
            root_to_tip[0, 0] = segment.getFrameToTip().M[0, 0];
            root_to_tip[0, 1] = segment.getFrameToTip().M[0, 1];
            root_to_tip[0, 2] = segment.getFrameToTip().M[0, 2];
            root_to_tip[1, 0] = segment.getFrameToTip().M[1, 0];
            root_to_tip[1, 1] = segment.getFrameToTip().M[1, 1];
            root_to_tip[1, 2] = segment.getFrameToTip().M[1, 2];
            root_to_tip[2, 0] = segment.getFrameToTip().M[2, 0];
            root_to_tip[2, 1] = segment.getFrameToTip().M[2, 1];
            root_to_tip[2, 2] = segment.getFrameToTip().M[2, 2];

            # Translation.
            root_to_tip[0, 3] = segment.getFrameToTip().p.x();
            root_to_tip[1, 3] = segment.getFrameToTip().p.y();
            root_to_tip[2, 3] = segment.getFrameToTip().p.z();

            base_to_root = base_to_root.dot(root_to_tip)

        return jac


class MotionPlanner:
    def __init__(self, name):
        self._action_server = actionlib.SimpleActionServer(
            name,
            abb_robot_sandbox.msg.PlanAction,
            execute_cb=self.callback,
            auto_start=False)
        self._action_server.start()

    def init(self):
        if not rospy.has_param('~robot_base_frame_id'):
            return False
        self._robot_base_frame_id = rospy.get_param(
            '~robot_base_frame_id')

        if not rospy.has_param('~robot_tip_frame_id'):
            return False
        self._robot_tip_frame_id = rospy.get_param(
            '~robot_tip_frame_id')

        if not rospy.has_param('~nominal_joint_vel'):
            return False
        self._nominal_joint_vel = rospy.get_param(
            '~nominal_joint_vel')

        # Initialize the robot kinematics (FK/IK).
        self._kinematics = RobotKinematics(
            self._robot_base_frame_id, self._robot_tip_frame_id)
        self._kinematics_helper = KinematicsHelper(
            self._kinematics._chain)

        if not rospy.has_param('~topics/vis'):
            return False
        vis_topic = rospy.get_param('~topics/vis')
        self._vis_pub = rospy.Publisher(vis_topic, Marker, queue_size=25)

        # Workspace parameters.
        # TODO Add error checking
        self._workspace_frame_id = rospy.get_param('~workspace/frame_id')
        self._workspace_center = rospy.get_param('~workspace/center')
        self._workspace_extents = rospy.get_param('~workspace/extents')

        # Interpolation parameters.
        self._interpolation_params = {}

        if not rospy.has_param('~interpolation/type'):
            return False
        self._interpolation_params['type'] = rospy.get_param(
            '~interpolation/type')

        if not rospy.has_param('~interpolation/linear_vel'):
            return False
        self._interpolation_params['linear_vel'] = rospy.get_param(
            '~interpolation/linear_vel')
        if not rospy.has_param('~interpolation/angular_vel'):
            return False
        self._interpolation_params['angular_vel'] = rospy.get_param(
            '~interpolation/angular_vel')

        if not rospy.has_param('~interpolation/time_step'):
            return False
        self._interpolation_params['time_step'] = rospy.get_param(
            '~interpolation/time_step')
        if not rospy.has_param('~interpolation/pseudoinverse_damping'):
            return False
        self._interpolation_params['pseudoinverse_damping'] = rospy.get_param(
            '~interpolation/pseudoinverse_damping')

        if not rospy.has_param('~interpolation/pos_error_tol'):
            return False
        self._interpolation_params['pos_error_tol'] = rospy.get_param(
            '~interpolation/pos_error_tol')
        if not rospy.has_param('~interpolation/orien_error_tol'):
            return False
        self._interpolation_params['orien_error_tol'] = rospy.get_param(
            '~interpolation/orien_error_tol')
        if not rospy.has_param('~interpolation/gains'):
            return False
        self._interpolation_params['gains'] = rospy.get_param(
            '~interpolation/gains')
        if not rospy.has_param('~interpolation/max_joint_vel'):
            return False
        self._interpolation_params['max_joint_vel'] = rospy.get_param(
            '~interpolation/max_joint_vel')

        return True

    def run(self):
        rospy.spin()

    def callback(self, goal):
        if goal.goal_type == PlanGoal.CONFIG_GOAL:
            rospy.loginfo(
                "[MotionPlanner] Received planning goal of type CONFIG_GOAL")

            rospy.logwarn("[MotionPlanner] CONFIG_GOAL not implemented!")
            result = PlanResult()
            result.result = PlanResult.FAILURE
            self._action_server.set_succeeded(result)
        elif goal.goal_type == PlanGoal.POSE_GOAL:
            rospy.loginfo(
                "[MotionPlanner] Received planning goal of type POSE_GOAL")
            result = PlanResult()
            result.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

            # Visualize the goal pose (z-axis only).
            self.pub_z_axis_marker(goal.goal_pose)

            # Visualize the workspace.
            self.pub_workspace_marker()

            if self._interpolation_params['type'] == 'taskspace':
                rospy.loginfo("[MotionPlanner] Using taskspace interpolation")
                result.trajectory.points = self.taskspace_interpolation(
                    goal.start_config, goal.goal_pose)
                self.pub_ee_trajectory_markers(
                    result.trajectory.points, subsample=10)
                result.result = PlanResult.SUCCESS
                self._action_server.set_succeeded(result)
            elif self._interpolation_params['type'] == 'jointspace':
                rospy.loginfo("[MotionPlanner] Using jointspace interpolation")

                goal_config = self._kinematics.ik(
                    [goal.goal_pose.position.x,
                     goal.goal_pose.position.y,
                     goal.goal_pose.position.z],
                    [goal.goal_pose.orientation.x,
                     goal.goal_pose.orientation.y,
                     goal.goal_pose.orientation.z,
                     goal.goal_pose.orientation.w])
                if goal_config is None:
                    rospy.logerr("[MotionPlanner] IK failed!")
                    result.result = PlanResult.FAILURE
                    self._action_server.set_succeeded(result)
                else:
                    goal_config = self.normalized_config(goal_config)
                    rospy.loginfo("[MotionPlanner] Goal config is {}".format(
                        goal_config))

                    max_time = -1.0
                    for i in range(6):
                        time = abs(goal_config[i] - goal.start_config.position[i]) / \
                            self._nominal_joint_vel
                        if time > max_time:
                            max_time = time

                    rospy.loginfo("[MotionPlanner] Trajectory time is {} seconds".
                                  format(max_time))
                    time_step = self._interpolation_params['time_step']

                    time_from_start = 0
                    while time_from_start < max_time:
                        point = JointTrajectoryPoint()
                        point.positions = 6 * [0.]
                        point.time_from_start = rospy.Duration(time_from_start)
                        p = time_from_start / max_time
                        for i in range(6):
                            point.positions[i] = goal.start_config.position[i] + \
                                p * (goal_config[i] - goal.start_config.position[i])

                        time_from_start += time_step
                        result.trajectory.points.append(point)

                    result.result = PlanResult.SUCCESS
                    self._action_server.set_succeeded(result)
            else:
                rospy.logerr("[MotionPlanner] Invalid interpolation type \"{}\"".
                             format(self._interpolation_params['type']))
                result = PlanResult()
                result.result = PlanResult.FAILURE
                self._action_server.set_succeeded(result)
        else:
            rospy.logerr("[MotionPlanner] Invalid goal type \"{}\"".format(
                goal.goal_type))
            result = PlanResult()
            result.result = PlanResult.FAILURE
            self._action_server.set_succeeded(result)

    def taskspace_interpolation(self, start_config, goal_pose):
        start_pos, start_orien = self._kinematics.fk([
            start_config.position[i] for i in range(6)])
        rospy.loginfo("[MotionPlanner] Start position: {}, orientation: {}".format(
            start_pos, start_orien))

        dist_to_goal = np.linalg.norm(np.array([
            goal_pose.position.x - start_pos[0],
            goal_pose.position.y - start_pos[1],
            goal_pose.position.z - start_pos[2]]))
        angular_dist_to_goal = self._kinematics_helper.shortest_angular_dist(
            start_orien, [goal_pose.orientation.x,
                          goal_pose.orientation.y,
                          goal_pose.orientation.z,
                          goal_pose.orientation.w])
        total_time = max(
            dist_to_goal / self._interpolation_params['linear_vel'],
            angular_dist_to_goal / self._interpolation_params['angular_vel'])
        rospy.loginfo("[MotionPlanner] Trajectory time: {} sec".format(
            total_time))
        time_step = self._interpolation_params['time_step']

        desired_poses = []
        num_desired_poses = int(total_time / time_step)
        for k in range(num_desired_poses):
            t = k / (num_desired_poses - 1.0)
            desired_pose = Pose()
            desired_pose.position.x = start_pos[0] + t * (
                goal_pose.position.x - start_pos[0])
            desired_pose.position.y = start_pos[1] + t * (
                goal_pose.position.y - start_pos[1])
            desired_pose.position.z = start_pos[2] + t * (
                goal_pose.position.z - start_pos[2])

            desired_orien = trans.quaternion_slerp(start_orien, [
                goal_pose.orientation.x, goal_pose.orientation.y,
                goal_pose.orientation.z, goal_pose.orientation.w], t)
            desired_pose.orientation.x = desired_orien[0]
            desired_pose.orientation.y = desired_orien[1]
            desired_pose.orientation.z = desired_orien[2]
            desired_pose.orientation.w = desired_orien[3]
            desired_poses.append(desired_pose)

        start_point = JointTrajectoryPoint()
        start_point.positions = start_config.position
        start_point.time_from_start = rospy.Duration(0)

        points = [start_point]

        goal_trans = np.eye(4)
        goal_trans[0:3, 0:3] = trans.quaternion_matrix([
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w
        ])[0:3, 0:3]
        goal_trans[0, 3] = goal_pose.position.x
        goal_trans[1, 3] = goal_pose.position.y
        goal_trans[2, 3] = goal_pose.position.z

        pos_error_tol = self._interpolation_params['pos_error_tol']
        orien_error_tol = self._interpolation_params['orien_error_tol']
        rospy.logdebug("[MotionPlanner] Error tol: pos: {}, orien: {}".format(
            pos_error_tol, orien_error_tol))

        pos_error = 1e9
        orien_error = 1e9

        time_from_start = 0

        lmda = self._interpolation_params['pseudoinverse_damping']
        lmda_mat = (lmda**2) * np.eye(6)

        gains = np.diag(self._interpolation_params['gains'])

        k = 0
        while abs(pos_error) > pos_error_tol or \
              abs(orien_error) > orien_error_tol:
            k += 1

            # Get the Jacobian from the current config.
            last_point = points[-1]
            jac = self._kinematics_helper.jac(
                [last_point.positions[i] for i in range(6)])
            rospy.logdebug("k: {}, jac: {}".format(k, jac))

            jac_pseudoinv_dls = jac.transpose().dot(np.linalg.inv(
                jac.dot(jac.transpose()) + lmda_mat))

            # Get the current pose.
            curr_pos, curr_orien = self._kinematics.fk(
                [last_point.positions[i] for i in range(6)])
            curr_trans = np.eye(4)
            curr_trans[0:3, 0:3] = trans.quaternion_matrix(curr_orien)[0:3, 0:3]
            curr_trans[0, 3] = curr_pos[0]
            curr_trans[1, 3] = curr_pos[1]
            curr_trans[2, 3] = curr_pos[2]

            # NOTE EXPERIMENTAL ---------------------------------------------- #
            idx = k - 1
            if idx >= len(desired_poses):
                idx = len(desired_poses) - 1

            desired_pose = desired_poses[idx]
            desired_trans = np.eye(4)
            desired_trans[0:3, 0:3] = trans.quaternion_matrix([
                desired_pose.orientation.x,
                desired_pose.orientation.y,
                desired_pose.orientation.z,
                desired_pose.orientation.w
            ])[0:3, 0:3]
            desired_trans[0, 3] = desired_pose.position.x
            desired_trans[1, 3] = desired_pose.position.y
            desired_trans[2, 3] = desired_pose.position.z
            err = np.linalg.inv(curr_trans).dot(desired_trans)
            # ---------------------------------------------------------------- #

            # err = np.linalg.inv(curr_trans).dot(goal_trans)

            # pos_error = np.linalg.norm(err[0:3, 3:])

            err_to_goal = np.linalg.inv(curr_trans).dot(goal_trans)
            pos_error = np.linalg.norm(err_to_goal[0:3, 3:])
            rospy.logdebug("POS ERROR: {}".format(pos_error))
            orien_error = self._kinematics_helper.shortest_angular_dist(
                curr_orien, [goal_pose.orientation.x,
                             goal_pose.orientation.y,
                             goal_pose.orientation.z,
                             goal_pose.orientation.w])
            rospy.logdebug("ORIEN ERROR: {}".format(orien_error))

            err_rot_homo = np.eye(4)
            err_rot_homo[0:3, 0:3] = err[0:3, 0:3]

            rot_err_angle, rot_err_axis, _ = trans.rotation_from_matrix(
                err_rot_homo)
            # print("k: {}, angle error: {}, axis error: {}".format(
            #     k, rot_err_angle, rot_err_axis))

            twist_err = np.zeros((6, 1))
            twist_err[0:3, 0:] = err[0:3, 3:]
            twist_err[3:, 0:] = rot_err_angle * rot_err_axis.reshape((3, 1))
            # print("  twist error: {}".format(twist_err))

            adj = self._kinematics_helper.adjoint(curr_trans)

            vel = jac_pseudoinv_dls.dot(gains.dot(adj.dot(twist_err)))
            # print("  vel: {}".format(vel))

            point = JointTrajectoryPoint()
            point.positions = 6 * [0.]
            point.time_from_start = rospy.Duration(time_from_start)

            # Clamp the actual velocity to the hard joint velocity limits.
            max_frac = 0.
            for i in range(6):
                if vel[i] > self._interpolation_params['max_joint_vel'][i]:
                    frac = (vel[i] -
                            self._interpolation_params['max_joint_vel'][i]) \
                            / vel[i]
                    max_frac = max(max_frac, frac)

            actual_vel = [(1 - max_frac) * vel[i] for i in range(6)]

            for i in range(6):
                point.positions[i] = last_point.positions[i] + \
                    time_step * actual_vel[i]

            time_from_start += time_step
            points.append(point)

        return points

    def normalized_config(self, config):
        return [normalize_angle(q) for q in config]

    def pub_z_axis_marker(self, pose):
        msg = Marker()
        msg.header.frame_id = self._robot_base_frame_id
        msg.id = 0
        msg.type = Marker.ARROW
        msg.ns = 'motion_planner/z_axis'
        msg.scale.x = 0.08
        msg.scale.y = 0.16
        msg.color.r = 0
        msg.color.g = 0
        msg.color.b = 1
        msg.color.a = 1

        world_to_local = PyKDL.Frame.Identity()
        world_to_local.p = PyKDL.Vector(
            pose.position.x, pose.position.y, pose.position.z)
        world_to_local.M = PyKDL.Rotation.Quaternion(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)

        first_point = PyKDL.Vector(0, 0, 0)
        second_point = PyKDL.Vector(0, 0, 0.2)

        first_point = world_to_local * first_point
        second_point = world_to_local * second_point

        p1 = Point()
        p1.x = first_point.x()
        p1.y = first_point.y()
        p1.z = first_point.z()
        msg.points.append(p1)

        p2 = Point()
        p2.x = second_point.x()
        p2.y = second_point.y()
        p2.z = second_point.z()
        msg.points.append(p2)

        self._vis_pub.publish(msg)

    def pub_workspace_marker(self):
        msg = Marker()
        msg.header.frame_id = self._workspace_frame_id
        msg.id = 0
        msg.type = Marker.LINE_LIST
        msg.ns = 'motion_planner/workspace'
        msg.scale.x = 0.05
        msg.color.r = 1
        msg.color.g = 0
        msg.color.b = 0
        msg.color.a = 0.35

        p1 = Point()
        p1.x = self._workspace_center[0] - 0.5 * self._workspace_extents[0]
        p1.y = self._workspace_center[1] - 0.5 * self._workspace_extents[1]
        p1.z = self._workspace_center[2] - 0.5 * self._workspace_extents[2]
        msg.points.append(p1)
        p2 = Point()
        p2.x = self._workspace_center[0] - 0.5 * self._workspace_extents[0]
        p2.y = self._workspace_center[1] - 0.5 * self._workspace_extents[1]
        p2.z = self._workspace_center[2] + 0.5 * self._workspace_extents[2]
        msg.points.append(p2)

        msg.points.append(p2)
        p3 = Point()
        p3.x = self._workspace_center[0] - 0.5 * self._workspace_extents[0]
        p3.y = self._workspace_center[1] + 0.5 * self._workspace_extents[1]
        p3.z = self._workspace_center[2] + 0.5 * self._workspace_extents[2]
        msg.points.append(p3)

        msg.points.append(p3)
        p4 = Point()
        p4.x = self._workspace_center[0] + 0.5 * self._workspace_extents[0]
        p4.y = self._workspace_center[1] + 0.5 * self._workspace_extents[1]
        p4.z = self._workspace_center[2] + 0.5 * self._workspace_extents[2]
        msg.points.append(p4)

        msg.points.append(p4)
        p5 = Point()
        p5.x = self._workspace_center[0] + 0.5 * self._workspace_extents[0]
        p5.y = self._workspace_center[1] - 0.5 * self._workspace_extents[1]
        p5.z = self._workspace_center[2] + 0.5 * self._workspace_extents[2]
        msg.points.append(p5)

        msg.points.append(p5)
        msg.points.append(p2)

        msg.points.append(p1)
        p6 = Point()
        p6.x = self._workspace_center[0] + 0.5 * self._workspace_extents[0]
        p6.y = self._workspace_center[1] - 0.5 * self._workspace_extents[1]
        p6.z = self._workspace_center[2] - 0.5 * self._workspace_extents[2]
        msg.points.append(p6)

        msg.points.append(p6)
        p7 = Point()
        p7.x = self._workspace_center[0] + 0.5 * self._workspace_extents[0]
        p7.y = self._workspace_center[1] + 0.5 * self._workspace_extents[1]
        p7.z = self._workspace_center[2] - 0.5 * self._workspace_extents[2]
        msg.points.append(p7)

        msg.points.append(p7)
        msg.points.append(p4)

        msg.points.append(p7)
        p8 = Point()
        p8.x = self._workspace_center[0] - 0.5 * self._workspace_extents[0]
        p8.y = self._workspace_center[1] + 0.5 * self._workspace_extents[1]
        p8.z = self._workspace_center[2] - 0.5 * self._workspace_extents[2]
        msg.points.append(p8)

        msg.points.append(p8)
        msg.points.append(p3)

        msg.points.append(p8)
        msg.points.append(p1)

        self._vis_pub.publish(msg)

    def pub_ee_trajectory_markers(self, configs, subsample=1):
        msg = Marker()
        msg.header.frame_id = self._workspace_frame_id
        msg.id = 0
        msg.type = Marker.SPHERE_LIST
        msg.ns = 'motion_planner/ee_traj'
        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.05
        msg.color.r = 0
        msg.color.g = 0.5
        msg.color.b = 0.5
        msg.color.a = 0.5

        for i in range(0, len(configs), subsample):
            p = Point()
            pos, _ = self._kinematics.fk(configs[i].positions)
            p.x = pos[0]
            p.y = pos[1]
            p.z = pos[2]
            msg.points.append(p)

        self._vis_pub.publish(msg)


def main():
    rospy.init_node('motion_planner')

    planner = MotionPlanner(rospy.get_name())
    if not planner.init():
        rospy.logerr("[MotionPlanner] Error: failed to initialize!")
        sys.exit(-1)

    planner.run()


if __name__ == '__main__':
    main()
