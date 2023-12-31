#!/usr/bin/env python3

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState


try:
    from math import pi, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as robotiq_outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as robotiq_inputMsg
from visualization_msgs.msg import Marker, MarkerArray


from visualization_msgs.msg import Marker, MarkerArray

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class UR5_Manipulator(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(UR5_Manipulator, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5_manipulator", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()


        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). The group name of the UR5 
        ## arm is set to "manipulator".
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # group_interface = moveit_commander.MoveGroupInterface(group_name, robot)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

       
        # reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.group_interface = group_interface
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.goal_states = dict() ### ---> Ex: {i:[x,y]}
        self.marker_states = dict() ### ---> Ex: {i:([x,y], color)}

        ###### Gripper init ######
        self.robotiq_gripper_pub = rospy.Publisher(
            "Robotiq2FGripperRobotOutput", robotiq_outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        self.robotiq_gripper_sub = rospy.Subscriber(
            "Robotiq2FGripperRobotInput", robotiq_inputMsg.Robotiq2FGripper_robot_input, self.gripper_status_callback)
        ### Camera subscriber init ###
        self.block_list_sub = rospy.Subscriber('/marker_array_topic', MarkerArray, self.block_list_callback)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
    def go_to_joint_state(self, joint_goal):

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        # We get the joint values from the group and change some of the values:
        # joint_goal = self.move_group.get_current_joint_values()
        # joint_goal[0] = 0
        # joint_goal[1] = -2*pi / 8
        # joint_goal[2] = 0
        # joint_goal[3] = -2*pi / 4
        # joint_goal[4] = 0
        # joint_goal[5] = 2*pi / 6  # 1/6 of a turn

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def tuck(self):
        tuck_pose = geometry_msgs.msg.Pose()
        tuck_pose.position.x = 0.229
        tuck_pose.position.y = 0.302
        tuck_pose.position.z = 0.417
        tuck_pose.orientation.x = 1.0

        self.go_to_pose_goal(tuck_pose)
    def wait_till_not_moving(self):
        while not rospy.is_shutdown():
            if all_close(self.joint_states, [0.0,0.0,0.0,0.0,0.0,0.0], 0.01):
                return
            rospy.sleep(0.08)

    def go_to_pose_goal(self, pose_goal):        
        """moves ur5 to target pose"""
        
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        #add a table to avoide collisions
        width = 0.05
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.z = -width/2
        box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.z = 0.11  # above the panda_hand frame
        self.box_name = "table"
        self.scene.add_box(self.box_name, box_pose, size=(2, 3, width))

        
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    def send_gripper_commad(self, rPR, rACT=1, rGTO=1, rATR=0, rSP=128, rFR=32):
        gripper_command = robotiq_outputMsg.Robotiq2FGripper_robot_output()
        gripper_command.rACT = rACT # must remain at 1, will activate upon being switched to one
        gripper_command.rGTO = rGTO # 1 means it is following the go to routine
        gripper_command.rATR = rATR # set to 1 for automatic release routine
        gripper_command.rPR = rPR
        gripper_command.rSP = rSP # 1/2 max speed
        gripper_command.rFR = rFR # 1/4 max force
        self.robotiq_gripper_pub.publish(gripper_command)
    def joint_state_callback(self, msg):
        self.joint_states = msg
    def gripper_status_callback(self, robotiq_inputMsg):
        self.gripper_status = robotiq_inputMsg
    def activate_gripper(self):
        #if gripper.status.rACT == 1: give warining to activate
        self.send_gripper_commad(rPR=0, rACT=0, rGTO=0, rATR=0, rSP=0, rFR=0)
        rospy.sleep(.2)
        self.send_gripper_commad(rPR=0, rACT=1, rGTO=1, rATR=0, rSP=128, rFR=48)
        rospy.sleep(2)

    def open_gripper(self, pr = 0):
        #if gripper.status.rACT == 0: give warining to activate
        self.send_gripper_commad(rPR=pr)
        #TODO wait till gripper stops
    
    def close_gripper(self):
        #if gripper.status.rACT == 0: give warining to activate
        self.send_gripper_commad(rPR=255, rFR=16)
        #TODO wait till gripper stops

    def block_list_callback(self, msg):
        self.block_list = msg
    # def get_block_list(self, duration = 5.0):

    #     start_time = rospy.Time.now()
    #     loop_duration = rospy.Duration(duration)  # 5 seconds duration
    #     start_time = rospy.Time.now()  # Getting the current time

    #     sampled_block_list = self.block_list.markers
    #     num_samples = 0
    #     while rospy.Time.now() < start_time + loop_duration:
    #         for i in range(len(sampled_block_list)):
    #             sampled_block_list[i].pose.position.x += self.block_list[i].pose.position.x
    #             sampled_block_list[i].pose.position.y += self.block_list[i].pose.position.y
    #         num_samples += 1
        
    #     sampled_block_list[i].pose.position.x = self.block_list[i].pose.position.x/num_samples
    #     sampled_block_list[i].pose.position.y = self.block_list[i].pose.position.y/num_samples
    #     return sampled_block_list

    def initialize_states(self):
        goal_i, marker_i = 0, 0
        self.rewards = dict()
        colors = set()
        for marker in self.block_list.markers:
            pos = [marker.pose.position.x, marker.pose.position.y]
            color = rgb_to_char(marker.color)
            if color == 'b':
                self.goal_states[goal_i] = pos
                goal_i += 1
            else:
                self.marker_states[marker_i] = (pos, color)
                marker_i += 1
            colors.add(color)
            
        for c in list(colors):
            self.rewards[c] = [None for _ in range(goal_i)]

        self.num_goals = goal_i
        self.num_markers = marker_i
            
        self.original_marker_states = copy.deepcopy(self.marker_states)

    def update_marker_state(self, i):
        # print(self.marker_states)
        c = self.marker_states[i][1]
        old_positions = []
        indices = []
        for k in self.marker_states:
            if self.marker_states[k][1] == c:
                old_positions.append(self.marker_states[k][0])
                indices.append(k)
        
        new_positions = []
        for marker in self.block_list.markers:
            if rgb_to_char(marker.color) == c:
                new_positions.append([marker.pose.position.x, marker.pose.position.y])
        min_distance, min_i, min_j = float("inf"), 0, 0
        for i in range(2):
            for j in range(2):
                dist = ((old_positions[i][0] - new_positions[j][0])**2 + (old_positions[i][1] - new_positions[j][1])**2)**0.5
                if dist < min_distance:
                    min_i, min_j = i, j
                    min_distance = dist
        
        self.marker_states[indices[min_i]] = (new_positions[min_j], c)
        self.marker_states[indices[1-min_i]] = (new_positions[1-min_j], c)
        # print(self.marker_states)
                
            

def test():
    ur5 = UR5_Manipulator()
    ur5.add_box()



    # ur5.robot.set_
    ur5.activate_gripper()
    
    input(
            "============ Press `Enter` to move to joint goal"
        )
    ur5.open_gripper()
    joint_goal = ur5.move_group.get_current_joint_values()
    joint_goal[0] = -1.61 # shoulder pan

    joint_goal[1] = -2.16 # sholder lift
    joint_goal[2] = -1.08 #  elbow
    joint_goal[3] = 4.84 
    joint_goal[4] = 1.60
    joint_goal[5] = 2.85
    ur5.go_to_joint_state(joint_goal)
    
    input(
            "============ Press `Enter` to grab"
        )
    ur5.close_gripper()

    

    input(
            "============ Press `Enter` to move to joint goal"
        )
    
    joint_goal = ur5.move_group.get_current_joint_values()
    joint_goal[0] = -1.13 # shoulder pan

    joint_goal[1] = -1.96 # sholder lift
    joint_goal[2] = -1.08 #  elbow
    joint_goal[3] = 4.84 
    joint_goal[4] = 1.60
    joint_goal[5] = 2.85
    ur5.go_to_joint_state(joint_goal)



    input(
            "============ Press `Enter` to move to joint goal"
        )
    
    joint_goal = ur5.move_group.get_current_joint_values()
    joint_goal[0] = -1.61 # shoulder pan

    joint_goal[1] = -2.16 # sholder lift
    joint_goal[2] = -1.08 #  elbow
    joint_goal[3] = 4.84 
    joint_goal[4] = 1.60
    joint_goal[5] = 2.85
    ur5.go_to_joint_state(joint_goal)

    input(
            "============ Press `Enter` to let go"
        )
    ur5.open_gripper()

    # input(
    #         "============ Press `Enter` to move to joint goal"
    #     )
    
    # joint_goal = ur5.move_group.get_current_joint_values()
    # joint_goal[0] = -1.08 

    # joint_goal[1] = -2.16
    # joint_goal[2] = -1.61
    # joint_goal[3] = -4.84
    # joint_goal[4] = 1.60
    # joint_goal[5] = 2.85
    # ur5.go_to_joint_state(joint_goal)
    return
    #intialize robot

    #get camera to world frame locations
    #get list of block locations
    #get list of goal locations
    #choose random block and goal
    
    block_pose = geometry_msgs.msg.Pose()
    #move block to goal
        #make frame that is centered on manipulator grab zone
        #transform from 
    ur5 = UR5_Manipulator()
    ur5.go_to_pose_goal(block_pose)
        #close gripper
    gripper_command = robotiq_outputMsg.Robotiq2FGripper_robot_output()
    gripper_command.rACT = 1
    gripper_command.rGTO = 1
    gripper_command.rSP = 128 # 1/2 max speed
    gripper_command.rFR = 64 # 1/4 max force

    #repeat until all blocks in goal

def test_gripper():
    ur5 = UR5_Manipulator()
    ur5.add_box()
    ur5.activate_gripper()

    input("Press `Enter` to open gripper")
    ur5.open_gripper()
    input("Press `Enter` to close gripper")
    ur5.close_gripper()

    input("Press `Enter` to move to ar tag")
    ur5.close_gripper()
    velocity_scaling_factor = 0.1  # Set your desired velocity scaling factor here
    ur5.move_group.set_max_velocity_scaling_factor(velocity_scaling_factor)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 1.0
    pose_goal.position.x = -0.19685
    pose_goal.position.y =  0.167
    pose_goal.position.z = 0.35
    ur5.go_to_pose_goal(pose_goal)

def rgb_to_char(color):
    if color.r == 1:
        return 'r'
    if color.b == 1:
        return 'b'
    if color.g == 1:
        return 'g'
def sort_blocks():
    try:
        ur5 = UR5_Manipulator()
        velocity_scaling_factor = 0.8  # Set your desired velocity scaling factor here
        ur5.move_group.set_max_velocity_scaling_factor(velocity_scaling_factor)
        ur5.add_box()
        ur5.activate_gripper()
        # current_pose = ur5.move_group.get_current_pose().pose
        # print(current_pose)
        ur5.initialize_states()

        ur5.rewards = [[None] * len(ur5.goal_states) for _ in range(2)] ### Hard-coded to handle two colors
        

        input("Press `Enter` to tuck")
        ur5.tuck()

        input("Press `Enter` to start sorting")
        ur5.open_gripper()
        
        #get list of goal location and number of blocks

        ur5.initialize_states()
        
        #count blocks already on each goal
        block_center_height = 0.05
        def move_block_to_goal(block_id, goal_id):
            position_a, color = ur5.marker_states[block_id]
            pose_block = geometry_msgs.msg.Pose()
            pose_block.orientation.x = 1.0
            pose_block.position.x = position_a[0]
            pose_block.position.y = position_a[1] 
            pose_block.position.z =  block_center_height
            
            id_num = int(block_id)
            offset = [[0, -0.05],[0.05, 0],[0, 0.05],[-0.05, 0]]
            position_g = ur5.goal_states[goal_id]
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = 1.0
            pose_goal.position.x = position_g[0] + offset[id_num][0]
            pose_goal.position.y = position_g[1] + offset[id_num][1]
            pose_goal.position.z = block_center_height
            move_block(pose_block, pose_goal)
            ur5.update_marker_state(block_id)

        def move_block_to_inital_location(block_id):
            position_a, color = ur5.marker_states[block_id]
            pose_block = geometry_msgs.msg.Pose()
            pose_block.orientation.x = 1.0
            pose_block.position.x = position_a[0]
            pose_block.position.y = position_a[1] 
            pose_block.position.z =  block_center_height
            
            position_b, c = ur5.original_marker_states[block_id]
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.x = 1.0
            pose_goal.position.x = position_b[0] 
            pose_goal.position.y = position_b[1]
            pose_goal.position.z = block_center_height
            move_block(pose_block, pose_goal)
            ur5.update_marker_state(block_id)
        
        def move_block(pose_a, pose_b): #location of block centroid, goal block centroid
            pose_goal = geometry_msgs.msg.Pose()
                        
            input("Press `Enter` move block")
            pose_goal.orientation.x = 1.0
            pose_goal.position.x = pose_a.position.x
            pose_goal.position.y = pose_a.position.y + 0.005 #tuning offset of 0.005
            pose_goal.position.z = pose_a.position.z + 0.29 + 0.05
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(2)

            pose_goal.position.z = pose_a.position.z + 0.29 
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(1)
            ur5.close_gripper()
            rospy.sleep(1.1)
            pose_goal.position.z = pose_a.position.z + 0.29  + 0.07
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(1)

            pose_goal.orientation.x = 1.0
            pose_goal.position.x = pose_b.position.x
            pose_goal.position.y = pose_b.position.y #+ 0.005
            pose_goal.position.z = pose_b.position.z + 0.29 + 0.07
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(2)
            pose_goal.position.z = pose_b.position.z + 0.29 + 0.005
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(1)
            ur5.open_gripper(145) # TODO tune
            rospy.sleep(1.5)
            pose_goal.position.z = pose_b.position.z + 0.29 + 0.05
            ur5.go_to_pose_goal(pose_goal)
            ur5.wait_till_not_moving()
            # rospy.sleep(1)
            ur5.open_gripper()
            ur5.tuck()
            ur5.wait_till_not_moving()
            rospy.sleep(1)

        
        for color in ['r', 'g']:
            goal_idx = 0
            for goal_id in ur5.goal_states:
                goal_idx += 1
                if ur5.rewards[color][goal_id]:
                    continue
                for block_id in ur5.marker_states.keys():
                    if ur5.marker_states[block_id][1] == color:
                        move_block_to_goal(block_id, goal_id)
                        x = input("Reward for most recent maneuver: ")
                        if x == '':
                            x = '0'
                        score = int(x)
                        ur5.rewards[color][goal_id] = score

                        if goal_idx >= ur5.num_goals:
                            move_block_to_inital_location(block_id)
                        break 
        
            
        print(ur5.rewards)
        
        for block_id in ur5.marker_states.keys():
            c = ur5.marker_states[block_id][1]
            goal_id = max(list(range(ur5.num_goals)), key = lambda x:ur5.rewards[c][x])
            if ur5.rewards[c][goal_id] > 0:
                move_block_to_goal(block_id, goal_id)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    

if __name__ == "__main__":

    sort_blocks()
