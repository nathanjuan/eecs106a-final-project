#!/usr/bin/env python
import cProfile
import math
import os
import sys

import rospy
from abb_robot_sandbox.col import CollisionBody
from abb_robot_sandbox.rrt import RRT
from abb_robot_sandbox.util import RobotKinematics
from abb_robot_sandbox.vis import Vis
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from rospkg import RosPack
from visualization_msgs.msg import *


def RRT_planner(kinematics, start_pos, goal_pos, start_orien, goal_orien, robot_cb, obstacle_cb=None, seed=None):
    if obstacle_cb and seed:
        rrt = RRT(start_pos, goal_pos, start_orien, goal_orien, robot_cb=robot_cb, obstacle_cbs=[obstacle_cb],
                  seed=seed)
    elif seed:
        rrt = RRT(start_pos, goal_pos, start_orien, goal_orien, robot_cb, seed=seed)
    else:
        rrt = RRT(start_pos, goal_pos, start_orien, goal_orien, robot_cb)
    # Plan with RRT
    rrt_data = rrt.plan(max_iters=10000)
    # Reconstruct path
    path = rrt.reconstruct_path(rrt_data)
    # Find shortcuts in path and add start config to path
    path = rrt.shortcutting(path)
    path.append(kinematics.ik(goal_pos))
    path.reverse()
    path.append(kinematics.ik(start_pos))
    path.reverse()
    # Do polynomial smoothing on path
    coeffs = rrt.get_path_coeffs(path, 0.5)
    smooth_path = rrt.get_configs(coeffs, 0.01, 0.5)
    return smooth_path


def on_run(feedback, profiling=False):
    # Verify that the start position is kinematically feasible.
    if kinematics.ik(start_pos) is None:
        rospy.logerr("From start position ({}, {}, {}), IK failed!".format(
            start_pos[0], start_pos[1], start_pos[2]))
        sys.exit(-1)

    # Get Unit Quaternion
    for i in range(len(start_orien)):
        start_orien[i] /= math.sqrt(
            start_orien[0] ** 2 + start_orien[1] ** 2 + start_orien[2] ** 2 + start_orien[3] ** 2)
    for i in range(len(orien_at_dash_pickup)):
        orien_at_dash_pickup[i] /= math.sqrt(
            orien_at_dash_pickup[0] ** 2 + orien_at_dash_pickup[1] ** 2 + orien_at_dash_pickup[2] ** 2 +
            orien_at_dash_pickup[3] ** 2)
    for i in range(len(orien_at_dash_pickup)):
        orien_at_dash_pickup[i] /= math.sqrt(
            orien_at_dash_pickup[0] ** 2 + orien_at_dash_pickup[1] ** 2 + orien_at_dash_pickup[2] ** 2 +
            orien_at_dash_pickup[3] ** 2)

    # Begin path to dashboard from starting position
    to_dash_smooth_path = RRT_planner(kinematics, start_pos, pos_at_dash_pickup, start_orien, orien_at_dash_pickup,
                                      robot_cb)

    # Combine dashboard_cb and robot_cb
    robot_cb.combine_col(dashboard_cb, 'link_6')
    # Begin path to goal from dashboard position
    to_goal_smooth_path = RRT_planner(kinematics, pos_at_dash_pickup, goal_pos, orien_at_dash_pickup, goal_orien,
                                      robot_cb)
    # Combine the different paths
    smooth_path = []
    smooth_path.extend(to_dash_smooth_path)
    smooth_path.extend(to_goal_smooth_path)
    if profiling:
        return
    rate = rospy.Rate(10)
    count = 0
    while not rospy.is_shutdown():
        if count >= len(smooth_path):
            count = 0
        vis.visualize_at_config_dashboard(smooth_path[count])
        # for i in range(len(path)):
        #     rgba = [0.5, 0.5, 0, 0.5]
        #     if i == 0:
        #         rgba = [1, 0, 0, 0.5]
        #     elif i == len(path_to_dash) - 1:
        #         rgba = [0, 1, 0, 0.5]
        #     vis.visualize_at_config_dashboard(path_to_dash[i], rgba=rgba, marker_id=i,
        #                                       marker_ns_prefix='rrt_demo_path_nodes')
        count += 1
        rate.sleep()


def profile(feedback):
    cProfile.run('on_run(None, True)')


def create_menu(int_marker):
    global server
    menu_handler = MenuHandler()
    run = menu_handler.insert("Run", callback=on_run)
    prof = menu_handler.insert("Profile", callback=profile)

    server.insert(int_marker, process_feedback)
    menu_handler.apply(server, int_marker.name)

    server.applyChanges()


def process_feedback(feedback):
    global start_pos, goal_pos, pos_at_dash_pickup, start_orien, goal_orien, orien_at_dash_pickup

    p = [feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z]
    o = [feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z,
         feedback.pose.orientation.w]

    if feedback.marker_name == "start_0":
        start_pos = p
        start_orien = o
        print("changed start")
    elif feedback.marker_name == "goal_0":
        goal_pos = p
        goal_orien = o
        print("changed goal")
    elif feedback.marker_name == "dash_pickup_0":
        pos_at_dash_pickup = p
        orien_at_dash_pickup = o
        print("changed dash_pickup")


if __name__ == '__main__':
    # initialize node, load dependencies
    rospy.init_node('rrt_demo')
    vis = Vis()
    rp = RosPack()
    kinematics = RobotKinematics()
    server = InteractiveMarkerServer("rrt_demo_marker_server")

    start_pos = [1, 0, 0.5]
    pos_at_dash_pickup = [1, 0, 0.5]
    goal_pos = [0, 0, 1]

    start_orien = [0, 0, 0, 1]
    orien_at_dash_pickup = [0, 0, 0, 1]
    goal_orien = [0, 0, 0, 1]

    # Get dashboard and robot collision spheres
    dashboard_cb = CollisionBody()
    robot_cb = CollisionBody()
    obstacle_cb = CollisionBody()
    path_to_package = rp.get_path('abb_robot_sandbox')
    dashboard_cb.load(os.path.join(path_to_package,
                                   'config/dashboard_collision_spheres.yaml'))
    robot_cb.load(os.path.join(path_to_package,
                               'config/irb4600_collision_spheres.yaml'))
    obstacle_cb.load(os.path.join(path_to_package,
                                  'config/obstacle_collision_spheres.yaml'))

    final_goal_marker = vis.visualize_sphere(goal_pos, 0.5, marker_ns="goal", interactive=True)
    create_menu(final_goal_marker)
    start_marker = vis.visualize_sphere(start_pos, 0.5, marker_ns="start", interactive=True)
    create_menu(start_marker)
    dash_pickup_marker = vis.visualize_sphere(pos_at_dash_pickup, 0.5, marker_ns="dash_pickup", interactive=True)
    create_menu(dash_pickup_marker)
    rospy.spin()
