import math
import yaml
from abb_robot_sandbox.util import TransformManager
from urdf_parser_py.urdf import URDF
import rospy
import random


class CollisionBody(object):
    """ Represents a rigid body in the environment with a collection of spheres """

    def __init__(self):
        self._frame_to_spheres = {}
        
    def has_frame(self, frame_name):
        return frame_name in self._frame_to_spheres

    def get_spheres(self, frame_name):
        """ Returns the list of spheres associated with specified coordinate frame
            NOTE: all positions are specified local to that coordinate frame """
        if frame_name not in self._frame_to_spheres:
            return []
        return self._frame_to_spheres[frame_name]

    def load(self, path_to_file):
        """ Loads a CollisionBody from a YAML file """
        """ Each sphere is a list with the structure [x, y, z, radius]"""
        with open(path_to_file, 'r') as f:
            config = yaml.safe_load(f)
            for frame in config['frames']:
                frame_name = frame['name']
                self._frame_to_spheres[frame_name] = []
                for sphere in frame['spheres']:
                    self._frame_to_spheres[frame_name].append(sphere['pos'][:3] + [sphere['radius'], 0]) # 0=original sphere in collision body, 1=added sphere due to combine_col()

    def combine_col(self, other_col, frame_to_put_other_col):
        other_col_spheres = other_col.get_spheres("link_1")
        for i in range(len(other_col_spheres)):
            other_col_spheres[i][4] = 1
        if frame_to_put_other_col in self._frame_to_spheres:
            self._frame_to_spheres[frame_to_put_other_col].extend(other_col_spheres)

    def separate_col(self, frame_with_other_col):
        other_col = {'link_1': []}
        if frame_with_other_col in self._frame_to_spheres:
            frame = self._frame_to_spheres[frame_with_other_col]
            i = 0
            while i < len(frame):
                if frame[i][4] == 1:
                    sphere = frame.pop(i)
                    sphere[4] = 0
                    other_col['link_1'].append(sphere)
                else:
                    i += 1
            self._frame_to_spheres[frame_with_other_col] = frame
        return other_col


class ColHandler(object):
    def __init__(self, robot_cb, joint_names):
        self._tm = TransformManager()
        self._robot_cb = robot_cb
        self._joint_names = joint_names

        urdf = URDF.from_parameter_server()        
        self._base_frame_id = urdf.get_root()
        self._link_names = [link.name for link in urdf.links]

    def col(self, sphere_1, sphere_2):
        # Find distance between sphere centers
        distance = math.sqrt(
            (sphere_1[0] - sphere_2[0]) ** 2 + (sphere_1[1] - sphere_2[1]) ** 2 + (sphere_1[2] - sphere_2[2]) ** 2)
        # If they are colliding return True, otherwise return False
	return distance <= sphere_1[3] + sphere_2[3]

    def col_bodies(self, robot_config, obstacle_body):
        """ Checks for collision between two CollisionBody objects """
        return self.get_col_bodies(robot_config, obstacle_body) != ()
    
    def get_col_bodies(self, robot_config, obstacle_body):
        """ Checks for collision between two CollisionBody objects and returns robot sphere colliding with obstacle body"""
        obstacle_spheres = obstacle_body.get_spheres('link_1')
        # Check links and the base link
        for i, frame_name in enumerate(self._link_names):
            robot_spheres = self._robot_cb.get_spheres(frame_name)
            for j, robot_sphere in enumerate(robot_spheres):
                robot_sphere_transformed = self._tm.transform_point(
                    robot_sphere[:3], self._joint_names, robot_config, frame_name, self._base_frame_id)
                if robot_sphere_transformed is not None:
                    robot_sphere_transformed.append(robot_sphere[3])
                    for k, obstacle_sphere in enumerate(obstacle_spheres):
                        if self.col(robot_sphere_transformed, obstacle_sphere):
                            rospy.loginfo("Collision between frame {} #{} and obstacle sphere #{}".format(frame_name, j, k))
                            return  (i , j)
                    if frame_name != "shoulder_link" and not (frame_name == "upper_arm_link" and j == 0) and robot_sphere_transformed[2] - robot_sphere[3] <= 0:
                        rospy.logdebug("Collision between table and link {} sphere #{}".format(frame_name, j))
                        return True
        
        return ()

    def col_links(self, robot_config, link_a, link_b):
        """Checks for collision between two links in the robot, returns True if there is collision, False if there
           isn't"""
        link_a_spheres = self._robot_cb.get_spheres(link_a)
        link_b_spheres = self._robot_cb.get_spheres(link_b)
        for i, link_a_sphere in enumerate(link_a_spheres):
            if link_a != self._base_frame_id:
                link_a_sphere_transformed = self._tm.transform_point(
                    link_a_sphere[:3], self._joint_names, robot_config, link_a, self._base_frame_id)
            else:
                link_a_sphere_transformed = link_a_sphere
            if link_a_sphere_transformed is not None:
                link_a_sphere_transformed.append(link_a_sphere[3])
                for j, link_b_sphere in enumerate(link_b_spheres):
                    if link_b != self._base_frame_id:
                        link_b_sphere_transformed = self._tm.transform_point(
                            link_a_sphere[:3], robot_config, link_b, self._base_frame_id)
                    else:
                        link_b_sphere_transformed = link_b_sphere
                    if link_b_sphere_transformed is not None:
                        link_b_sphere_transformed.append(link_b_sphere[3])
                        if self.col(link_a_sphere_transformed, link_b_sphere_transformed):
                            rospy.logdebug("[col_links]: Collision between {} (Sphere #{}) and {} (Sphere #{})".format(link_a, i, link_b, j))
                            return True
        return False

    def col_links_v2(self, link_a, link_b, spheres):
        """Checks if two links are in collision. Takes in link a's index and link b's index and a list of pre-transformed collision spheres."""
        for i, link_a_sphere in enumerate(spheres[link_a]):
            if link_a_sphere is not None:
                for j, link_b_sphere in enumerate(spheres[link_b]):
                    if link_b_sphere is not None:
                        if self.col(link_a_sphere, link_b_sphere):
                            rospy.loginfo("[col_links]: Collision between {} (Link {}, Sphere #{}) and {} (Link {}, Sphere #{})".format(link_a_sphere, link_a, i, link_b_sphere, link_b, j))
                            return True
        return False

    def get_spheres_at_config(self, robot_config):
        """Gets the collision spheres that have been transformed to the base_link coordinate space for all links at the robot config specified."""
        spheres = {}
        link = self._robot_cb.get_spheres(self._base_frame_id)
        spheres[self._base_frame_id] = link
        for i, link_name in enumerate(self._link_names):
            if link_name != self._base_frame_id:
                link = self._robot_cb.get_spheres(link_name)
                spheres[link_name] = []
                for j, sphere in enumerate(link):
                    spheres[link_name].append(self._tm.transform_point(sphere[:3], self._joint_names, robot_config, link_name, self._base_frame_id))
                    spheres[link_name][j].append(sphere[3])
        return spheres

class ConfigurationValidator(object):
    def __init__(self, robot_cb, manipulator_joint_names, obstacle_cbs=None, finger_joint_name=None, seed=None):
        """
        robot_cb: CollisionBody of robot
        joint_names: list of joint names to be passed into TransformManager
        obstacle_cbs (optional): CollisionBody(s) of obstacles
        seed (optional): Specify seed when sampling configurations
        append_joint_poss (optional): Joint angles to append to configurations passed into is_valid (eg. finger_joint in Robotiq parallel grippers)
        """
        self._robot_cb = robot_cb
        self._obstacle_cbs = obstacle_cbs if obstacle_cbs is not None else []
        if seed is not None:
            random.seed(seed)
        
        self.urdf = URDF.from_parameter_server()
        self._joint_names = manipulator_joint_names
        self._link_names = [link.name for link in self.urdf.links]
        self._base_frame_id = self.urdf.get_root()
        finger_joint_names = [finger_joint_name] if finger_joint_name is not None else []
        
        self._col_handler = ColHandler(self._robot_cb, self._joint_names + finger_joint_names)

        self._joint_limits = []
        for joint_name in manipulator_joint_names:
            joint = self.urdf.joint_map[joint_name]
            self._joint_limits.append((max(joint.limit.lower, -math.pi), min(joint.limit.upper, math.pi)))
            
    def set_finger_joint_poss(self, finger_joint_poss):
        self._finger_joint_poss = finger_joint_poss
        
    def _is_link_b_child(self, link_a, link_b):
        if link_a not in self.urdf.child_map:
            return False
        for child in self.urdf.child_map[link_a]:
            if child[1] == link_b:
                return True
        return False

    def _self_collision(self, config):
        spheres = self._col_handler.get_spheres_at_config(config)
        for i, link_a in enumerate(self._link_names):
            for j, link_b in enumerate(self._link_names[i+1:]):
                links_to_not_check_between = ["wrist_1_link", "wrist_2_link", "wrist_3_link", "flange_interface", "upper_ft_interface", "robotiq_arg2f_base_link"]
                if not (link_a in links_to_not_check_between and link_b in links_to_not_check_between) and not (link_a == "forearm_link" and link_b == "wrist_2_link") and not self._is_link_b_child(link_a, link_b):
                    if self._col_handler.col_links_v2(link_a, link_b, spheres):
                        return True
        return False

    def is_valid(self, config, do_append_joints=False):
        """ Checks if the configuration is valid (i.e. not in collision) and that the joint limits are satisfied"""
        robot_config = config + (self._finger_joint_poss if do_append_joints else [])
        for obstacle_cb in self._obstacle_cbs:
            if self._col_handler.col_bodies(robot_config, obstacle_cb):
                return False
        # Check joint limits
        for i in range(len(config)):
            if not self._joint_limits[i][0] <= config[i] <= self._joint_limits[i][1]:
                return False
        # Check for self-collision
        return not self._self_collision(robot_config)
