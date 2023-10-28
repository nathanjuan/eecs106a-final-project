#!/usr/bin/env python

import os
import rospy
from rospkg import RosPack
import sys
import actionlib
import abb_robot_sandbox
from abb_robot_sandbox.msg import PlanGoal, PlanFeedback, PlanResult
from abb_robot_sandbox.util import RobotKinematics, reorder_joints
from abb_robot_sandbox.col import CollisionBody, ConfigurationValidator
from abb_robot_sandbox.rrt_v2 import RRT
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
import PyKDL
from angles import shortest_angular_distance, normalize_angle
from urdf_parser_py.urdf import URDF


class RRTMotionPlanner:
    def __init__(self, name):
        self._action_server = actionlib.SimpleActionServer(
            name,
            abb_robot_sandbox.msg.PlanAction,
            execute_cb=self.callback,
            auto_start=False)
        self._action_server.start()

    def init(self):
        if not rospy.has_param('robot_configuration/base_frame_id'):
            return False
        self._robot_base_frame_id = rospy.get_param(
            'robot_configuration/base_frame_id')

        if not rospy.has_param('robot_configuration/tip_frame_id'):
            return False
        self._robot_tip_frame_id = rospy.get_param(
            'robot_configuration/tip_frame_id')

        urdf = URDF.from_parameter_server()
        self._joint_names_to_tip = [joint.name for joint in urdf.joints if joint.type == "revolute"]
        
        if not rospy.has_param('robot_configuration/manipulator_joint_names'):
            return False
        self._manipulator_joint_names = rospy.get_param(
            'robot_configuration/manipulator_joint_names')
            
        if not rospy.has_param('robot_configuration/joint_names_to_tip'):
            return False
        self._joint_names_to_tip = rospy.get_param(
            'robot_configuration/joint_names_to_tip')
            
        if not rospy.has_param('robot_configuration/has_finger_joint'):
            return False
        self._has_finger_joint = rospy.get_param('robot_configuration/has_finger_joint')
        
        if not rospy.has_param('robot_configuration/finger_joint_name'):
            return False
        self._finger_joint_name = rospy.get_param('robot_configuration/finger_joint_name')

        if not rospy.has_param('~nominal_joint_vel'):
            return False
        self._nominal_joint_vel = rospy.get_param(
            '~nominal_joint_vel')
            
        if not rospy.has_param('robot_configuration/collision_spheres'):
            return False
        robot_col_spheres_file = rospy.get_param('robot_configuration/collision_spheres')

        if not rospy.has_param('~obstacle_collision_spheres'):
            return False
        obs_col_spheres_file = rospy.get_param('~obstacle_collision_spheres')

        if not rospy.has_param('~smooth_path_total_duration'):
            return False
        self._smooth_path_tot_duration = float(rospy.get_param('~smooth_path_total_duration'))

        if not rospy.has_param('~smooth_path_timestep'):
            return False
        self._smooth_path_dt = float(rospy.get_param('~smooth_path_timestep'))

        if not rospy.has_param('~topics/vis'):
            return False
        vis_topic = rospy.get_param('~topics/vis')
        self._vis_pub = rospy.Publisher(vis_topic, Marker, queue_size=25)

        # Initialize the robot kinematics (FK/IK)
        self._kinematics = RobotKinematics(self._robot_tip_frame_id)

        # Load collision bodies
        rp = RosPack()
        path_to_package = rp.get_path('abb_robot_sandbox')

        self._robot_cb = CollisionBody()
        self._robot_cb.load(os.path.join(path_to_package, robot_col_spheres_file))

        self._obstacle_cbs = [CollisionBody()]
        self._obstacle_cbs[0].load(os.path.join(path_to_package, obs_col_spheres_file))

        self._rrt_config_validator = ConfigurationValidator(self._robot_cb, self._manipulator_joint_names, self._obstacle_cbs, self._finger_joint_name if self._has_finger_joint else None)
        self._kin_config_validator = ConfigurationValidator(self._robot_cb, self._joint_names_to_tip, self._obstacle_cbs, self._finger_joint_name if self._has_finger_joint else None)

        return True

    def callback(self, goal):
        result = PlanResult()
        
        # Get finger_joint info (if present)
        if goal.has_finger_joint:
            finger_joint_config = list(goal.finger_joint_config.position)
            finger_joint_names = list(goal.finger_joint_config.name)
            self._rrt_config_validator.set_finger_joint_poss(finger_joint_config)
            self._kin_config_validator.set_finger_joint_poss(finger_joint_config)
        else:
            finger_joint_config = []
            finger_joint_names = []
        # Make sure joints are in correct order (by the name parameter in JointState)
        start_config = reorder_joints(goal.start_config.position, goal.start_config.name, self._manipulator_joint_names)
        if goal.goal_type == PlanGoal.CONFIG_GOAL:
            rospy.loginfo("[RRTMotionPlanner] Received planning goal of type CONFIG_GOAL")
            goal_config = reorder_joints(goal.goal_config.position, goal.goal_config.name, self._manipulator_joint_names)

            # Create Pose msg for visualization
            goal_pos, goal_orien = self._kinematics.fk(self._robot_joint_names, goal_config + finger_joint_config)
            goal_pose = Pose()
            goal_pose.position.x = goal_pos[0]
            goal_pose.position.y = goal_pos[1]
            goal_pose.position.z = goal_pos[2]
            goal_pose.orientation.x = goal_orien[0]
            goal_pose.orientation.y = goal_orien[1]
            goal_pose.orientation.z = goal_orien[2]
            goal_pose.orientation.w = goal_orien[3]

            # Visualize the goal pose (z-axis only).
            self.pub_z_axis_marker(goal_pose)
        elif goal.goal_type == PlanGoal.POSE_GOAL:
            rospy.loginfo("[RRTMotionPlanner] Received planning goal of type POSE_GOAL")
            rospy.loginfo("[RRTMotionPlanner] Goal Position and Orientation: {}, {}".format(
                [goal.goal_pose.position.x,
                 goal.goal_pose.position.y,
                 goal.goal_pose.position.z],
                [goal.goal_pose.orientation.x,
                 goal.goal_pose.orientation.y,
                 goal.goal_pose.orientation.z,
                 goal.goal_pose.orientation.w]))

            # Visualize the goal pose (z-axis only).
            self.pub_z_axis_marker(goal.goal_pose)

            # IK returns config with mimic joints, but we only want manipulator config, so we need to remove the additional joint angles
            goal_config = self._kinematics.ik_valid(self._kin_config_validator,
                                              [goal.goal_pose.position.x, goal.goal_pose.position.y,
                                               goal.goal_pose.position.z],
                                              orien=[goal.goal_pose.orientation.x, goal.goal_pose.orientation.y,
                                               goal.goal_pose.orientation.z, goal.goal_pose.orientation.w])[:len(start_config)]
            if goal_config is None:
                rospy.logerr("[RRTMotionPlanner] IK failed for goal_config!")
                result.result = PlanResult.FAILURE
                self._action_server.set_succeeded(result)
                return
            else:
                goal_config = self.normalized_config(goal_config)
        else:
            rospy.logerr("Invalid goal type \"{}\"".format(goal.goal_type))
            result.result = PlanResult.FAILURE
            self._action_server.set_succeeded(result)
            return
        rospy.loginfo("[MotionPlanner] Goal config is {}".format(
            goal_config))

        # Run RRT to get smooth path
        smooth_path = self.run_rrt(start_config, goal_config, self._smooth_path_tot_duration, self._smooth_path_dt)

        if not smooth_path:
            rospy.logerr("[RRTMotionPlanner] Could not find RRT plan with specified start and goal!")
            result.result = PlanResult.FAILURE
            self._action_server.set_succeeded(result)
        else:
            rospy.loginfo("[MotionPlanner] Trajectory time is {} seconds".format(self._smooth_path_tot_duration))

            time_from_start = 0
            # Copy the smooth_path into JointTrajectoryPoints and add to result.trajectory
            for pos in smooth_path:
                point = JointTrajectoryPoint()
                point.positions = pos[:]
                point.time_from_start = rospy.Duration(time_from_start)

                result.trajectory.points.append(point)
                time_from_start += 0.1

            result.result = PlanResult.SUCCESS
            self._action_server.set_succeeded(result)

    def normalized_config(self, config):
        return [normalize_angle(q) for q in config]

    def pub_z_axis_marker(self, pose):
        msg = Marker()
        msg.header.frame_id = self._robot_base_frame_id
        msg.id = 0
        msg.type = Marker.ARROW
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

    def run_rrt(self, start_config, goal_config, total_duration, smooth_path_dt):
        # Initialize RRT
        rrt = RRT(start_config, goal_config, self._rrt_config_validator, self._robot_cb, obstacle_cbs=self._obstacle_cbs)

        # Plan with RRT
        path = rrt.plan(max_iterations=10000)
        if path == "failure":
            return []

        # Find shortcuts in path
        rrt.shortcutting(path)

        # Generate smooth path
        coeffs = rrt.get_path_coeffs(path, total_duration)
        smooth_path = rrt.get_configs(coeffs, smooth_path_dt, total_duration)
        return smooth_path

    def run(self):
        rospy.spin()


def main():
    rospy.init_node("rrt_motion_planner")

    planner = RRTMotionPlanner(rospy.get_name())
    if not planner.init():
        rospy.logerr("[RRTMotionPlanner] Error: failed to initialize!")
        sys.exit(-1)

    planner.run()


if __name__ == '__main__':
    main()
