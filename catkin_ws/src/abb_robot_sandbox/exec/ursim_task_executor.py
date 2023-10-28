#!/usr/bin/env python
import rospy
import sys
import actionlib
import abb_robot_sandbox
from abb_robot_sandbox.msg import PlanGoal, PlanResult, ControlGoal, ControlResult
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tf2_ros
import PyKDL
# from robotiq_vacuum_grippers_control.msg import \
#     _RobotiqVacuumGrippers_robot_output as GripperOutputMessage
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as ParallelGripperInputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as ParallelGripperOutputMsg
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint


class URSimTaskExecutor:
    def __init__(self):
        self._motion_planning_client = actionlib.SimpleActionClient(
            "rrt_motion_planner",
            abb_robot_sandbox.msg.PlanAction)
        self._joint_traj_client = actionlib.SimpleActionClient(
            "scaled_pos_joint_traj_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._task_index = 0

    def init(self):
        rospy.loginfo("[TaskExecutor] Waiting for motion planner...")
        self._motion_planning_client.wait_for_server()

        if not rospy.has_param('task_executor/freq'):
            return False
        self._freq = rospy.get_param('task_executor/freq')

        if not rospy.has_param('task_executor/world_frame_id'):
            return False
        self._world_frame_id = rospy.get_param('task_executor/world_frame_id')

        if not rospy.has_param('task_executor/topics/joint_states'):
            return False
        self._joint_states_topic = rospy.get_param('task_executor/topics/joint_states')   

        if not rospy.has_param('task_executor/services/start_vacuum_gripper'):
            return False
        self._start_vacuum_gripper_service = rospy.ServiceProxy(
            rospy.get_param('task_executor/services/start_vacuum_gripper'), Trigger)

        if not rospy.has_param('task_executor/services/stop_vacuum_gripper'):
            return False
        self._stop_vacuum_gripper_service = rospy.ServiceProxy(
            rospy.get_param('task_executor/services/stop_vacuum_gripper'), Trigger)
            
        robotiq_gripper_status_topic = rospy.get_param(
            '~robotiq_gripper_status_topic', 'Robotiq2FGripperRobotInput')
        self._robotiq_gripper_status_sub = rospy.Subscriber(
            robotiq_gripper_status_topic,
            ParallelGripperInputMsg,
            self.parallel_gripper_status_callback)
        self._parallel_gripper_current_status = None

        self._robotiq_parallel_gripper_cmd_pub = rospy.Publisher(
              'Robotiq2FGripperRobotOutput', ParallelGripperOutputMsg)
        
        if not rospy.has_param('robot_configuration/manipulator_joint_names'):
            return False
        self._manipulator_joint_names = rospy.get_param('robot_configuration/manipulator_joint_names')
        
        if not rospy.has_param('robot_configuration/has_finger_joint'):
            return False
        self._has_finger_joint = rospy.get_param('robot_configuration/has_finger_joint')
        
        if not rospy.has_param('robot_configuration/finger_joint_name'):
            return False
        self._finger_joint_name = rospy.get_param('robot_configuration/finger_joint_name')

        if not rospy.has_param('task_executor/tasks'):
            return False
        self._tasks = rospy.get_param('task_executor/tasks')

        return True

    def run(self):
        self._rate = rospy.Rate(self._freq)

        while not rospy.is_shutdown():
            if self.execute_task():
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

            self._rate.sleep()

    def execute_task(self):
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
                    "[TaskExecutor] Failed to lookup transform from {} to {}"
                    .format(self._world_frame_id, task["frame_id"]))
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

            rospy.loginfo("[TaskExecutor] Waiting for current manipulator config...")
            
            goal.has_finger_joint.data = self._has_finger_joint
            
            manipulator_joint_state_msg = None
            finger_joint_state_msg = None
            while True:
                joint_state_msg = rospy.wait_for_message(self._joint_states_topic, JointState)
                if self._has_finger_joint and joint_state_msg.name[0] == self._finger_joint_name:
                    finger_joint_state_msg = joint_state_msg
                elif len(joint_state_msg.name) == len(self._manipulator_joint_names):
                    manipulator_joint_state_msg = joint_state_msg
                    
                if manipulator_joint_state_msg and (finger_joint_state_msg or not self._has_finger_joint):
                    break
                
            goal.start_config = manipulator_joint_state_msg
            goal.finger_joint_config = finger_joint_state_msg

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

            # After receiving plan from rrt_motion_planner, send to joint_trajectory_action_server
            try:
                ctrl_msg = FollowJointTrajectoryGoal()
                ctrl_msg.trajectory = result.trajectory
                ctrl_msg.trajectory.joint_names = self._manipulator_joint_names
                ctrl_msg.path_tolerance = [] # JointTolerance()
                ctrl_msg.goal_tolerance = [] # JointTolerance()
                for name in self._manipulator_joint_names:
                    tol = JointTolerance()
                    tol.name = name
                    tol.position = 0
                    tol.velocity = 0
                    tol.acceleration = 0
                    ctrl_msg.path_tolerance.append(tol)
                    ctrl_msg.goal_tolerance.append(tol)

                self._joint_traj_client.send_goal(ctrl_msg)
                rospy.loginfo("[TaskExecutor] Sent trajectory to robot")
                self._joint_traj_client.wait_for_result()
                rospy.loginfo("[TaskExecutor] Finished running trajectory")
                # Done!
            except rospy.ROSInterruptException:
                rospy.logerr("[TaskExecutor] Control request interrupted!")
                return False
        elif task["type"] == "activate_parallel_gripper":
            rospy.loginfo("[TaskExecutor] Activating parallel gripper...")
            self.activate_parallel_gripper()
        elif task["type"] == "set_parallel_gripper_pos":
            if "pos" in task and 0 <= task["pos"] <= 255:
                self.set_parallel_gripper_pos(task["pos"])
            else:
                rospy.logerror("[TaskExecutor)] 'pos' argument for set_parallel_gripper_pos is missing or not in range of [0, 255]!")
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
            try:
                while not success:
                    # NOTE In Python3, use input() instead of raw_input()
                    res = raw_input("[TaskExecutor] Input \"{}\" to continue: ".format(task["input"]))
                    if res == task["input"]:
                        success = True
            except:
                sys.exit()
        elif task["type"] == "wait_for_duration":
            if not "sec" in task:
                rospy.logerr(
                    "[TaskExecutor] Must specify seconds for wait_for_duration!")
                return False
            rospy.loginfo("[TaskExecutor] Waiting for {} sec...".
                          format(task["sec"]))
            rospy.sleep(task["sec"])
        elif task["type"] == "move_to_config":
            try:
                ctrl_msg = FollowJointTrajectoryGoal()
				
                goal_point = JointTrajectoryPoint()
                goal_point.positions = task["config"]
                goal_point.velocities = [0] * 6
                goal_point.accelerations = [0] * 6
                goal_point.time_from_start = rospy.Duration(task["duration"])

                ctrl_msg.trajectory.points = [goal_point]
                ctrl_msg.trajectory.joint_names = self._manipulator_joint_names
                ctrl_msg.path_tolerance = [] # JointTolerance()
                ctrl_msg.goal_tolerance = [] # JointTolerance()
                for name in self._manipulator_joint_names:
                    tol = JointTolerance()
                    tol.name = name
                    tol.position = 0
                    tol.velocity = 0
                    tol.acceleration = 0
                    ctrl_msg.path_tolerance.append(tol)
                    ctrl_msg.goal_tolerance.append(tol)

                self._joint_traj_client.send_goal(ctrl_msg)
                rospy.loginfo("[TaskExecutor] Sent trajectory to robot")
                # self._joint_traj_client.wait_for_result()
                rospy.loginfo("[TaskExecutor] Finished running trajectory")
                # Done!
            except rospy.ROSInterruptException:
                rospy.logerr("[TaskExecutor] Control request interrupted!")
                return False
        else:
            rospy.logerr(
                "[TaskExecutor] For task {}, unknown task type \"{}\"!".format(
                    self._task_index, task["type"]))
            return False

        return True
        
    def parallel_gripper_status_callback(self, msg):
        self._parallel_gripper_current_status = msg

    def activate_parallel_gripper(self):
        self._parallel_gripper_output_msg = ParallelGripperOutputMsg()
        self._parallel_gripper_output_msg.rACT = 0
        self._parallel_gripper_output_msg.rGTO = 0
        self._parallel_gripper_output_msg.rATR = 0
        self._parallel_gripper_output_msg.rPR = 0
        self._parallel_gripper_output_msg.rSP = 0
        self._parallel_gripper_output_msg.rFR = 0
        self._robotiq_parallel_gripper_cmd_pub.publish(self._parallel_gripper_output_msg)
        
        while self._parallel_gripper_current_status.gACT != 0:
            self._rate.sleep()

        self._parallel_gripper_output_msg.rACT = 1
        self._parallel_gripper_output_msg.rGTO = 1
        self._parallel_gripper_output_msg.rSP = 255
        self._parallel_gripper_output_msg.rFR = 150
        self._robotiq_parallel_gripper_cmd_pub.publish(self._parallel_gripper_output_msg) 
        # Wait until status is 3 (gripper has been activated)
        while self._parallel_gripper_current_status.gSTA != 3:
            self._rate.sleep()
        print("[TaskExecutor] Robotiq parallel gripper activated")
        
    def set_parallel_gripper_pos(self, pos):
        self._parallel_gripper_output_msg.rPR = pos
        self._robotiq_parallel_gripper_cmd_pub.publish(self._parallel_gripper_output_msg)
        
        # Wait until gOBJ is 3 (gripper is at requested position)
        while self._parallel_gripper_current_status.gOBJ != 3:
            self._rate.sleep()
        print("[TaskExecutor] Robotiq parallel gripper activated")
        

    def start_robotiq_vacuum_gripper(self):
        # rospy.loginfo(
        #     "[TaskExecutor] Sending start command to Robotiq vacuum gripper")
        # msg = GripperOutputMessage.RobotiqVacuumGrippers_robot_output()
        # msg.rACT = 1
        # msg.rMOD = 0
        # msg.rGTO = 1
        # msg.rATR = 0
        # msg.rPR = 0
        # msg.rSP = 150
        # msg.rFR = 50
        # self._robotiq_vacuum_gripper_cmd_pub.publish(msg)
        pass

    def stop_robotiq_vacuum_gripper(self):
        # rospy.loginfo(
        #     "[TaskExecutor] Sending stop command to Robotiq vacuum gripper")
        # msg = GripperOutputMessage.RobotiqVacuumGrippers_robot_output()
        # msg.rACT = 1
        # msg.rMOD = 0
        # msg.rGTO = 1
        # msg.rATR = 0
        # msg.rPR = 255
        # msg.rSP = 150
        # msg.rFR = 50
        # self._robotiq_vacuum_gripper_cmd_pub.publish(msg)
        pass


def main():
    rospy.init_node('ursim_task_executor')

    executor = URSimTaskExecutor()
    if not executor.init():
        rospy.logerr("[URSimTaskExecutor] Error: failed to initialize!")
        sys.exit(-1)

    executor.run()


if __name__ == '__main__':
    main()
