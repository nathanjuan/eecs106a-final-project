#!/usr/bin/env python

import rospy
import sys
import actionlib
import abb_robot_sandbox
from abb_robot_sandbox.msg import PlanGoal, PlanResult, ControlGoal, ControlResult
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tf2_ros
import PyKDL
from robotiq_vacuum_grippers_control.msg import \
    _RobotiqVacuumGrippers_robot_output as GripperOutputMessage


class TaskExecutor:
    def __init__(self):
        self._motion_planning_client = actionlib.SimpleActionClient(
            "rrt_motion_planner", abb_robot_sandbox.msg.PlanAction)  # TODO: Make an argument to specify the planning type (rrt or regular motion planner)
        self._controller_client = actionlib.SimpleActionClient(
            "controller", abb_robot_sandbox.msg.ControlAction)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._task_index = 0

    def init(self):
        rospy.loginfo("[TaskExecutor] Waiting for motion planner...")
        self._motion_planning_client.wait_for_server()

        rospy.loginfo("[TaskExecutor] Waiting for controller...")
        self._controller_client.wait_for_server()

        if not rospy.has_param('~freq'):
            return False
        self._freq = rospy.get_param('~freq')

        if not rospy.has_param('~world_frame_id'):
            return False
        self._world_frame_id = rospy.get_param('~world_frame_id')

        if not rospy.has_param('~topics/joint_states'):
            return False
        self._joint_states_topic = rospy.get_param('~topics/joint_states')

        if not rospy.has_param('~services/start_vacuum_gripper'):
            return False
        self._start_vacuum_gripper_service = rospy.ServiceProxy(
            rospy.get_param('~services/start_vacuum_gripper'), Trigger)

        if not rospy.has_param('~services/stop_vacuum_gripper'):
            return False
        self._stop_vacuum_gripper_service = rospy.ServiceProxy(
            rospy.get_param('~services/stop_vacuum_gripper'), Trigger)

        self._robotiq_vacuum_gripper_cmd_pub = rospy.Publisher(
            'RobotiqVacuumGrippersRobotOutput',
            GripperOutputMessage.RobotiqVacuumGrippers_robot_output)

        # Load the tasks.
        if not rospy.has_param('~tasks'):
            return False
        self._tasks = rospy.get_param('~tasks')

        return True

    def run(self):
        rate = rospy.Rate(self._freq)

        while not rospy.is_shutdown():
            if self.execute_next_task():
                rospy.loginfo("[TaskExecutor] Task {} complete!".format(
                    self._task_index))
                self._task_index += 1
            else:
                rospy.logwarn(
                    "[TaskExecutor] Task {} failed! Trying again...".format(
                        self._task_index))

            if self._task_index >= len(self._tasks):
                rospy.loginfo("[TaskExecutor] All tasks complete! Exiting...")
                break

            rate.sleep()

    def execute_next_task(self):
        rospy.loginfo("[TaskExecutor] Executing task {}...".format(
            self._task_index))

        if self._task_index < 0 or self._task_index >= len(self._tasks):
            rospy.logwarn("[TaskExecutor] Task index {} out of bounds!".format(
                self._task_index))
            return False

        task = self._tasks[self._task_index]
        if task["type"] == "move_to_goal_pose":
            # Move the end-effector to the goal pose, in the specified frame.
            try:
                world_to_frame_trans = self._tf_buffer.lookup_transform(
                    self._world_frame_id, task["frame_id"], rospy.Time())
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logerr(
                    "[TaskExecutor] Failed to lookup transform from {} to {}".
                    format(self._world_frame_id, task["frame_id"]))
                return False

            world_to_frame = PyKDL.Frame.Identity()
            world_to_frame.p = PyKDL.Vector(
                world_to_frame_trans.transform.translation.x,
                world_to_frame_trans.transform.translation.y,
                world_to_frame_trans.transform.translation.z)
            world_to_frame.M = PyKDL.Rotation.Quaternion(
                world_to_frame_trans.transform.rotation.x,
                world_to_frame_trans.transform.rotation.y,
                world_to_frame_trans.transform.rotation.z,
                world_to_frame_trans.transform.rotation.w)

            frame_to_pose = PyKDL.Frame.Identity()
            frame_to_pose.p = PyKDL.Vector(
                task["pose"]["pos"][0],
                task["pose"]["pos"][1],
                task["pose"]["pos"][2])
            if len(task["pose"]["orien"]) == 4:
                # Quaternion.
                frame_to_pose.M = PyKDL.Rotation.Quaternion(
                    task["pose"]["orien"][0],
                    task["pose"]["orien"][1],
                    task["pose"]["orien"][2],
                    task["pose"]["orien"][3])
            elif len(task["pose"]["orien"]) == 3:
                # RPY (Euler).
                frame_to_pose.M = PyKDL.Rotation.RPY(
                    task["pose"]["orien"][0],
                    task["pose"]["orien"][1],
                    task["pose"]["orien"][2])
            else:
                # TODO Throw error
                pass

            world_to_pose = world_to_frame * frame_to_pose

            # Request a motion plan to the goal pose.
            goal = PlanGoal()
            goal.goal_type = PlanGoal.POSE_GOAL
            goal.goal_pose.position.x = world_to_pose.p.x()
            goal.goal_pose.position.y = world_to_pose.p.y()
            goal.goal_pose.position.z = world_to_pose.p.z()
            orien = world_to_pose.M.GetQuaternion()
            goal.goal_pose.orientation.x = orien[0]
            goal.goal_pose.orientation.y = orien[1]
            goal.goal_pose.orientation.z = orien[2]
            goal.goal_pose.orientation.w = orien[3]

            rospy.loginfo("[TaskExecutor] Waiting for current config...")
            joint_state_msg = rospy.wait_for_message(
                self._joint_states_topic, JointState)
            goal.start_config = joint_state_msg

            try:
                self._motion_planning_client.send_goal(goal)
                rospy.loginfo("[TaskExecutor] Waiting for plan...")
                self._motion_planning_client.wait_for_result()

                result = self._motion_planning_client.get_result()
                if result.result != PlanResult.SUCCESS:
                    rospy.logerr("[TaskExecutor] Motion planning failed!")
                    return False

                rospy.loginfo("[TaskExecutor] Result: {}".format(result.result))
                rospy.loginfo("[TaskExecutor] Plan contains {} waypoints".format(
                    len(result.trajectory.points)))
            except rospy.ROSInterruptException:
                rospy.logerr("[TaskExecutor] Motion plan request interrupted!")
                return False

            ctrl_goal = ControlGoal()
            ctrl_goal.trajectory = result.trajectory

            try:
                self._controller_client.send_goal(ctrl_goal)
                rospy.loginfo(
                    "[TaskExecutor] Waiting for trajectory to finish...")
                self._controller_client.wait_for_result()

                ctrl_result = self._controller_client.get_result()
                if ctrl_result.result != ControlResult.SUCCESS:
                    rospy.logerr("[TaskExecutor] Controller failed!")
                    return False

                # Done!
            except rospy.ROSInterruptException:
                rospy.logerr("[TaskExecutor] Control request interrupted!")
                return False
        elif task["type"] == "start_vacuum_gripper":
            rospy.loginfo("[TaskExecutor] Starting vacuum gripper...")
            if "robotiq" in task and task["robotiq"]:
                self.start_robotiq_vacuum_gripper()
            else:
                try:
                    res = self._start_vacuum_gripper_service()
                    if not res.success:
                        return False
                except rospy.ServiceException as e:
                    rospy.logerr("[TaskExecutor] Failed to start vacuum gripper: {}".
                                 format(e))
                    return False
        elif task["type"] == "stop_vacuum_gripper":
            rospy.loginfo("[TaskExecutor] Stopping vacuum gripper...")
            if "robotiq" in task and task["robotiq"]:
                self.stop_robotiq_vacuum_gripper()
            else:
                try:
                    res = self._stop_vacuum_gripper_service()
                    if not res.success:
                        return False
                except rospy.ServiceException as e:
                    rospy.logerr("[TaskExecutor] Failed to stop vacuum gripper: {}".
                                 format(e))
                    return False
        elif task["type"] == "wait_for_user_input":
            if not "input" in task:
                rospy.logerr(
                    "[TaskExecutor] Must specify input for wait_for_user_input!")
                return False

            success = False
            while not success:
                # NOTE In Python3, use input() instead of raw_input()
                res = raw_input("[TaskExecutor] Input \"{}\" to continue: ".
                                format(task["input"]))
                if res == task["input"]:
                    success = True
        elif task["type"] == "wait_for_duration":
            if not "sec" in task:
                rospy.logerr(
                    "[TaskExecutor] Must specify seconds for wait_for_duration!")
                return False
            rospy.loginfo("[TaskExecutor] Waiting for {} sec...".
                          format(task["sec"]))
            rospy.sleep(task["sec"])
        else:
            rospy.logerr(
                "[TaskExecutor] For task {}, unknown task type \"{}\"!".format(
                    self._task_index, task["type"]))
            return False

        return True

    def start_robotiq_vacuum_gripper(self):
        rospy.loginfo(
            "[TaskExecutor] Sending start command to Robotiq vacuum gripper")
        msg = GripperOutputMessage.RobotiqVacuumGrippers_robot_output()
        msg.rACT = 1
        msg.rMOD = 0
        msg.rGTO = 1
        msg.rATR = 0
        msg.rPR = 0
        msg.rSP = 150
        msg.rFR = 50
        self._robotiq_vacuum_gripper_cmd_pub.publish(msg)

    def stop_robotiq_vacuum_gripper(self):
        rospy.loginfo(
            "[TaskExecutor] Sending stop command to Robotiq vacuum gripper")
        msg = GripperOutputMessage.RobotiqVacuumGrippers_robot_output()
        msg.rACT = 1
        msg.rMOD = 0
        msg.rGTO = 1
        msg.rATR = 0
        msg.rPR = 255
        msg.rSP = 150
        msg.rFR = 50
        self._robotiq_vacuum_gripper_cmd_pub.publish(msg)


def main():
    rospy.init_node('task_executor')

    executor = TaskExecutor()
    if not executor.init():
        rospy.logerr("[TaskExecutor] Error: failed to initialize!")
        sys.exit(-1)

    executor.run()


if __name__ == '__main__':
    main()
