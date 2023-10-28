#!/usr/bin/env python

import os
import rospy
import sys
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import abb_robot_sandbox.util as util
from urdf_parser_py.urdf import URDF
from abb_robot_sandbox.col import CollisionBody
from rospkg import RosPack
from abb_robot_sandbox.util import TransformManager, reorder_joints, RobotKinematics


class DummySim:
    def __init__(self):
        pass

    def init(self):
        if not rospy.has_param('~freq'):
            return False
        self._freq = rospy.get_param('~freq')

        if not rospy.has_param('~robot_config'):
            return False
        self._robot_config = rospy.get_param('~robot_config')

        if not rospy.has_param('robot_configuration/collision_spheres'):
            return False
        self._robot_col_spheres_file = rospy.get_param('robot_configuration/collision_spheres')
        
        if not rospy.has_param('~obstacle_collision_spheres'):
            return False
        self._obs_col_spheres_file = rospy.get_param('~obstacle_collision_spheres')

        if not rospy.has_param('~topics/joint_states'):
            return False
        self._joint_state_sub = rospy.Subscriber(
            rospy.get_param('~topics/joint_states'), JointState,
            self.joints_callback)

        if not rospy.has_param("robot_configuration/finger_joint_name"):
            self._finger_joint_name = None
        else:
            self._finger_joint_name = rospy.get_param("robot_configuration/finger_joint_name")

        if not rospy.has_param("robot_configuration/manipulator_joint_names"):
            return False
        self._manipulator_joint_names = rospy.get_param("robot_configuration/manipulator_joint_names")
        
        urdf = URDF.from_parameter_server()
        self._base_link = urdf.get_root()
        self._link_names = [link.name for link in urdf.links]
        
        self._tm = TransformManager()
        
        self._blue = [0, 0, 1, 1]
        self._yellow = [1, 1, 0, 1]

#       if not rospy.has_param("robot_configuration/base_frame_id"):
#           return False
#       self._robot_base_frame_id = rospy.get_param("robot_configuration/base_frame_id")

        # For obstacles
        self._obstacle_pub = rospy.Publisher('/obstacle_spheres', MarkerArray, queue_size=1)
        
        self._robot_pub = rospy.Publisher('/robot_spheres', MarkerArray, queue_size=1)

        self._obstacle_cb = CollisionBody()
        self._robot_cb = CollisionBody()
        rp = RosPack()
        path_to_package = rp.get_path('abb_robot_sandbox')
        
        self._obstacle_cb.load(os.path.join(path_to_package, self._obs_col_spheres_file))
        self._robot_cb.load(os.path.join(path_to_package, self._robot_col_spheres_file))

        return True

    def joints_callback(self, message):
        # Update the robot's configuration
        if len(message.position) == 6:
            if self._finger_joint_name is not None:
                given_config = list(message.position) + [self._robot_config[-1]]
                given_joint_names = list(message.name) + [self._finger_joint_name]
                self._robot_config = reorder_joints(
                    given_config, given_joint_names, self._manipulator_joint_names + [self._finger_joint_name])
            else:
                self._robot_config = reorder_joints(
                    list(message.position), list(message.name), self._manipulator_joint_names)
        elif self._finger_joint_name is not None and message.name == [self._finger_joint_name]:
            self._robot_config[-1] = message.position[0]

    def run(self):
        rate = rospy.Rate(self._freq)

        # joints_msg = JointState()
        # joints_msg.name = self._robot_joint_names
        # joints_msg.position = self._robot_config
        # joints_msg.velocity = self._robot_joint_count * [0.]
        # joints_msg.effort = self._robot_joint_count * [0.]

        # Create MarkerArray of obstacles from collision body
        obstacle_msg = MarkerArray()
        for i, sphere in enumerate(self._obstacle_cb.get_spheres("link_1")):
            obstacle_sphere = Marker()
            obstacle_sphere.header.frame_id = "world"
            obstacle_sphere.header.stamp = rospy.Time.now()
            obstacle_sphere.lifetime = rospy.Duration(0)
            obstacle_sphere.ns = "obstacle_spheres"
            obstacle_sphere.id = i
            obstacle_sphere.type = Marker.SPHERE
            obstacle_sphere.pose = util.get_pose(sphere[:3], [0, 0, 0, 1])
            obstacle_sphere.scale = util.get_scale([sphere[3]*2] * 3)
            obstacle_sphere.color = util.get_color([1, 0, 0, 1])
            obstacle_msg.markers.append(obstacle_sphere)

        while not rospy.is_shutdown():
            self._obstacle_pub.publish(obstacle_msg)

            for i, link_name in enumerate(self._link_names):
                if not self._robot_cb.has_frame(link_name):
                    continue
                robot_cb_msg = MarkerArray()
                for j, sphere in enumerate(self._robot_cb.get_spheres(link_name)):
                    sphere_base_link_pos = self._tm.transform_point(
                        sphere[:3], self._manipulator_joint_names + [self._finger_joint_name], self._robot_config, link_name, self._base_link)
                    vis_sphere = Marker()
                    vis_sphere.header.frame_id = "world"
                    vis_sphere.header.stamp = rospy.Time.now()
                    vis_sphere.lifetime = rospy.Duration(0)
                    vis_sphere.ns = "robot_cb_" + link_name
                    vis_sphere.type = Marker.SPHERE
                    vis_sphere.id = j
                    vis_sphere.pose = util.get_pose(sphere_base_link_pos, [0, 0, 0, 1])
                    vis_sphere.scale = util.get_scale([sphere[3] * 2] * 3)
                    vis_sphere.color = util.get_color(self._yellow if sphere[4] == 1 else self._blue)
                    robot_cb_msg.markers.append(vis_sphere)
                self._robot_pub.publish(robot_cb_msg)
            rate.sleep()


def main():
    rospy.init_node('dummy_sim')

    sim = DummySim()
    if not sim.init():
        rospy.logerr("[DummySim] Error: failed to initialize!")
        sys.exit(-1)

    sim.run()


if __name__ == '__main__':
    main()
