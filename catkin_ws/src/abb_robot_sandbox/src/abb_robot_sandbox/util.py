import kdl_parser_py.urdf  # sudo apt install ros-kinetic-kdl-parser-py
from PyKDL import *
import rospy
from urdf_parser_py.urdf import URDF, Joint
from rospkg import RosPack
import os
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA
from angles import shortest_angular_distance, normalize_angle
import math
import random


class TransformManager(object):
    """ Manages transforming points between different coordinate frames """

    def __init__(self):
        self._world_frame = None
        self._tree = None
        self._frame_to_fk_solver = {}
        self._frame_to_joints = {}
        self._revolute_joint_list = []
        self._revolute_joints = {}

        try:
            ok, self._tree = kdl_parser_py.urdf.treeFromParam('robot_description')
            if ok:
                rospy.loginfo('Ok! Parsed tree with {} joints and {} segments'.
                                  format(self._tree.getNrOfJoints(),
                                         self._tree.getNrOfSegments()))
                # Get URDF from parameter server
                urdf = URDF.from_parameter_server()
                # Get robot base frame id from URDF
                self._world_frame = urdf.get_root()
                # Get joint names and mimic from URDF and filter out all non-revolute joints (ie. fixed joints)
                for joint in urdf.joints:
                    if joint.type == "revolute":
                        self._revolute_joint_list.append(joint.name)
                        self._revolute_joints[joint.name] = joint.mimic
                # Get link names from URDF
                link_names = [link.name for link in urdf.links]
                
                for i, name in enumerate(link_names):
                    base_to_link_chain = self._tree.getChain(self._world_frame, name)
                    self._frame_to_fk_solver[name] = ChainFkSolverPos_recursive(base_to_link_chain)
                    self._frame_to_joints[name] = []
                    for j in range(base_to_link_chain.getNrOfSegments()):
                        joint = base_to_link_chain.getSegment(j).getJoint()
                        if joint.getTypeName() != u"None":
                            self._frame_to_joints[name].append(joint.getName())
            else:
                rospy.logerr('Not ok!')
        except Exception as err:
            rospy.logerr('[TFMgr] Error: {}'.format(err))

    def transform_point(self, point_in_A, given_joint_names, given_robot_config, frame_A_name, frame_B_name):
        """
        Transforms a point specified in frame A's coordinates to frame B's coordinates.

        :param point: (x, y, z) coordinates of the point in frame A
        :type point: list
        :param robot_config: 6 joint angles of the robot's current configuration
        :type robot_config: list
        :param frame_A_name: name of coordinate frame A
        :type frame_A_name: string
        :param frame_B_name: name of coordinate frame B
        :type frame_B_name: string
        :rtype: list

        """
        # Convert given robot config to one with mimic joints
        config_map = dict(zip(given_joint_names, given_robot_config))
#        print("Given joint names: ", given_joint_names)
 #       print(given_robot_config)
        robot_config = []
        for name in self._revolute_joint_list:
            mimic = self._revolute_joints[name]
            if name in given_joint_names:
                robot_config.append(config_map[name])
            elif mimic is not None and mimic.joint in given_joint_names:
                robot_config.append(config_map[mimic.joint] * mimic.multiplier + mimic.offset)
            else:
                rospy.logerr("[TFMgr] Could not find joint named {} in robot config!".format(name))
                return None
        
        p_in_A = Vector(point_in_A[0], point_in_A[1], point_in_A[2])

        if frame_A_name is not self._world_frame and \
                frame_A_name not in self._frame_to_fk_solver:
            rospy.logerr('Frame A ({}) not found!'.format(frame_A_name))
            return None
        if frame_B_name is not self._world_frame and \
                frame_B_name not in self._frame_to_fk_solver:
            rospy.logerr('Frame B ({}) not found!'.format(frame_B_name))
            return None

        # Get the world to frame A transform.
        world_to_A = Frame.Identity()

        if frame_A_name is not self._world_frame:
            config_A = JntArray(len(self._frame_to_joints[frame_A_name]))
            for i in range(len(self._frame_to_joints[frame_A_name])):
                config_A[i] = robot_config[i]

            self._frame_to_fk_solver[frame_A_name].JntToCart(config_A, world_to_A)

        # Get the world to frame B transform.
        world_to_B = Frame.Identity()

        if frame_B_name is not self._world_frame:
            config_B = JntArray(len(self._frame_to_joints[frame_B_name]))
            for i in range(len(self._frame_to_joints[frame_B_name])):
                config_B[i] = robot_config[i]

            self._frame_to_fk_solver[frame_B_name].JntToCart(config_B, world_to_B)

        B_to_A = world_to_B.Inverse() * world_to_A
        p_in_B = B_to_A * p_in_A
        return [p_in_B.x(), p_in_B.y(), p_in_B.z()]


class RobotKinematics(object):
    def __init__(self, ee_frame_id):
        """
        ee_frame_id: the id of the end effector's frame
        """
        self._ee_frame = ee_frame_id
        self._world_frame = None
        self._tree = None
        self._chain = None
        self._fk_solver = None
        self._ik_vel_solver = None
        self._ik_solver = None
        self._jac_solver = None
        self._revolute_joint_list = []
        self._revolute_joints = {}
        try:
            ok, self._tree = kdl_parser_py.urdf.treeFromParam('robot_description')
            
            # Get URDF from parameter server
            urdf = URDF.from_parameter_server()
            # Get robot base frame id from URDF
            self._world_frame = URDF.from_parameter_server().get_root()
            # Get joint names and mimic from URDF and filter out all non-revolute joints (ie. fixed joints)
            for joint in urdf.joints:
                if joint.type == "revolute":
                    self._revolute_joint_list.append(joint.name)
                    self._revolute_joints[joint.name] = joint.mimic
            # Get link names from URDF
            link_names = [link.name for link in urdf.links]
            
            if ok:
                rospy.loginfo('Ok! Parsed tree with {} joints and {} segments'.
                              format(self._tree.getNrOfJoints(),
                                     self._tree.getNrOfSegments()))
                self._chain = self._tree.getChain(self._world_frame, self._ee_frame)
                self._fk_solver = ChainFkSolverPos_recursive(self._chain)
                self._ik_vel_solver = ChainIkSolverVel_pinv(self._chain)
                self._ik_solver = ChainIkSolverPos_NR(
                    self._chain, self._fk_solver, self._ik_vel_solver)
                self._jac_solver = ChainJntToJacSolver(
                    self._chain)
            else:
                rospy.logerr('Not ok!')
        except Exception as err:
            rospy.logerr('[RobotKin] Error: {}'.format(err))
            
    def given_config_to_full_config(given_joint_names, given_robot_config):
        config_map = dict(zip(given_joint_names, given_robot_config))
        robot_config = []
        for name in self._revolute_joint_list:
            mimic = self._revolute_joints[name]
            if name in given_joint_names:
                robot_config.append(config_map[name])
            elif mimic is not None and mimic.joint in config_map:
                robot_config.append(config_map[mimic.joint] * mimic.multiplier + mimic.offset)
            else:
                rospy.logerr("[RobotKinematics] Could not find joint named {} in robot config!".format(name))
                return None
            
    def normalized_config(self, config):
        return [normalize_angle(q) for q in config]

    def fk_pos(self, given_joint_names, given_config):
        """ Returns the position of the end-effector for the specified configuration. """
        config = given_config_to_full_config(given_joint_names, given_config)
        
        jnts = JntArray(len(config))
        for i in range(len(config)):
            jnts[i] = config[i]

        ee_in_world = Frame.Identity()
        self._fk_solver.JntToCart(jnts, ee_in_world)
        return [ee_in_world.p.x(), ee_in_world.p.y(), ee_in_world.p.z()]

    def jac(self, given_joint_names, given_config):
        config = given_config_to_full_config(given_joint_names, given_config)
    
        jnts = JntArray(len(config))
        for i in range(len(config)):
            jnts[i] = config[i]

        return self._jac_solver.JntToJac(jnts)

    def fk(self, given_joint_names, given_config):
        """ Returns the position and orientation of the end-effector for the specified configuration. """
        config = given_config
        
        ee_pos = [0, 0, 0]
        ee_orien = [0, 0, 0, 1]  # NOTE: rotation as a quaternion

        jnts = JntArray(len(config))
        for i in range(len(config)):
            jnts[i] = config[i]
        ee_in_world = Frame.Identity()
        self._fk_solver.JntToCart(jnts, ee_in_world)

        ee_pos = [ee_in_world.p.x(), ee_in_world.p.y(), ee_in_world.p.z()]
        ee_orien = ee_in_world.M.GetQuaternion()
        return ee_pos, ee_orien

    def ik(self, pos, orien=None, seed=None):
        """ Attempts to find the joint angles that take the end effector to the desired pose. """     
        ee_in_world = Frame(Vector(pos[0], pos[1], pos[2]))
        if orien is not None:
            ee_in_world.M = Rotation.Quaternion(
                orien[0], orien[1], orien[2], orien[3])

        seed_jnts = JntArray(self._tree.getNrOfJoints())
        for i in range(self._tree.getNrOfJoints()):
            seed_jnts[i] = seed[i] if seed else 0.

        soln_jnts = JntArray(self._tree.getNrOfJoints())
        res = self._ik_solver.CartToJnt(seed_jnts, ee_in_world, soln_jnts)
        if res >= 0:
            return [soln_jnts[i] for i in range(self._tree.getNrOfJoints())]

        rospy.logerr("Error: [ik] failed!")
        return None

    def ik_valid(self, config_validator, pos, orien=None, seed=None, max_iterations=100):
        """ Runs IK until robot configuration is valid (no self-collision, exceeding joint angles, or obstacle collisions)"""
        joint_names = []
        for i in range(self._chain.getNrOfSegments()):
            joint = self._chain.getSegment(i).getJoint()
            if joint.getTypeName() != u"None":
                joint_names.append(joint.getName())
        
        ee_in_world = Frame(Vector(pos[0], pos[1], pos[2]))
        if orien is not None:
            ee_in_world.M = Rotation.Quaternion(
                orien[0], orien[1], orien[2], orien[3])

        for i in range(max_iterations):
            seed_jnts = JntArray(self._chain.getNrOfJoints())
            for i in range(self._chain.getNrOfJoints()):
                seed_jnts[i] = seed[i] if seed else random.uniform(-math.pi, math.pi)
            
            soln_jnts = JntArray(self._chain.getNrOfJoints())
            res = self._ik_solver.CartToJnt(seed_jnts, ee_in_world, soln_jnts)
            res_list = self.normalized_config([soln_jnts[i] for i in range(self._chain.getNrOfJoints())])
            
            print("ik res", res_list)
            if res >= 0 and config_validator.is_valid(res_list, True):
                return res_list
        
        rospy.logerr("Error: [ik_valid] failed!")
        return None
            

def get_pose(pos, orien):
    if len(pos) != 3:
        raise ValueError("Position array must be of length 3 but was of length {}".format(len(pos)))
    if len(orien) != 4:
        raise ValueError("Orientation array must of length 4 but was of length {}".format(len(orien)))

    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = orien[0]
    pose.orientation.y = orien[1]
    pose.orientation.z = orien[2]
    pose.orientation.w = orien[3]

    return pose


def get_color(color_list):
    if len(color_list) != 4:
        raise ValueError("Color array must of length 4 but was of length {}".format(len(color_list)))

    color = ColorRGBA()
    color.r = color_list[0]
    color.g = color_list[1]
    color.b = color_list[2]
    color.a = color_list[3]
    return color


def get_scale(scale_list):
    if len(scale_list) != 3:
        raise ValueError("Scale array must of length 3 but was of length {}".format(len(scale_list)))

    scale = Vector3()
    scale.x = scale_list[0]
    scale.y = scale_list[1]
    scale.z = scale_list[2]
    return scale


def reorder_joints(joints_in, ordering_in, ordering_out):
    """
    Reorders the joints according to a desired ordering.

    :param joints_in: list of joint angles in an ordering specified by ordering_in
    :param ordering_in: ordered list of joint names, corresponding to the order of joints_in
    :param ordering_out: ordered list of joint names, corresponding to the desired order of the joints
    """
    joints_out = len(joints_in) * [0.0]

    for i in range(len(joints_in)):
        idx = -1
        for j in range(len(ordering_out)):
            if ordering_in[i] == ordering_out[j]:
                idx = j
                break
        joints_out[idx] = joints_in[i]

    return joints_out
