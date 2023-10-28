#!/usr/bin/env python

import rospy
from abb_robot_sandbox.col import CollisionBody
from abb_robot_sandbox.rrt import RRT
# from abb_robot_sandbox.rrt import sample_config
from abb_robot_sandbox.util import TransformManager
from abb_robot_sandbox.vis import Vis
from abb_robot_sandbox.util import RobotKinematics
from rospkg import RosPack
import os
import random
import sys


def main():
    rospy.init_node('test_rrt')
    tm = TransformManager()
    vis = Vis()
    kinematics = RobotKinematics()

    # Load the robot's collision body from the YAML config file.
    robot_cb = CollisionBody()
    obstacle_cb = CollisionBody()
    rp = RosPack()
    path_to_package = rp.get_path('abb_robot_sandbox')
    robot_cb.load(os.path.join(path_to_package,
                               'config/robot_collision_spheres.yaml'))
    obstacle_cb.load(os.path.join(path_to_package,
                                  'config/obstacle_collision_spheres.yaml'))

    # Choose a random start (x, y, z)-position for the end-effector.
    # seed = 42
    # seed = 43
    # seed = 44
    # seed = 45  # Causes strange behavior!
    # seed = 46  # Also causes strange behavior!
    seed = 60
    random.seed(seed)
    xyz_limits = [
        (-1, 1), # (min_x, max_x) limits on x-coordinate
        (-1, 1), # (min_y, max_y) limits on y-coordinate
        (-1, 1)  # (min_z, max_z) limits on z-coordinate
    ]
    start_pos = [
        random.randrange(xyz_limits[i][0], xyz_limits[i][1]) for i in range(3)]

    # Verify that the start position is kinematically feasible.
    if kinematics.ik(start_pos) is None:
        rospy.logerr("From start position ({}, {}, {}), IK failed!".format(
            start_pos[0], start_pos[1], start_pos[2]))
        sys.exit(-1)

    # Set start_pos and goal_pos
    # start_pos = sample_config()
    # start_pos = [1.1, 1, 1]
    print("Start Position: " + str(start_pos))
    # Set the random seed (so that all runs will produce the exact same samples).
    rrt = RRT(start_pos, goal_pos=[1, 1, 1], robot_cb=robot_cb, obstacle_cbs=[obstacle_cb], seed=42)

    # Begin RRT planning
    rrt_data = rrt.plan(max_iters=5000)
    rospy.loginfo("Done planning!")

    # Reconstruct path
    path = rrt.reconstruct_path(rrt_data)
    rospy.loginfo("Finding shortcuts in path.")

    # Find shortcuts in path and add start config to path
    path = rrt.shortcutting(path)
    path.append(kinematics.ik([1, 1, 1]))
    path.reverse()
    path.append(kinematics.ik(start_pos))
    path.reverse()

    # Get smooth path
    coeffs = rrt.get_path_coeffs(path, 0.5)
    smooth_path = rrt.get_configs(coeffs, 0.01, 0.5)
    print("Final Path Length: " + str(len(path)))
    print("Final Path Nodes: " + str(path))

    rate = rospy.Rate(10)
    path_counter = 0
    smooth_path_counter = 0
    while not rospy.is_shutdown():
        # Visualize the start configuration.
        vis.visualize_at_config(kinematics.ik(start_pos), rgba=[1, 0, 0, 0.25],
                                marker_id=0, marker_ns_prefix="start_config")

        # Visualize goal position and configuration.
        vis.visualize_sphere(
            tm.transform_point([0, 0, 0], rrt._goal_config, "link_6", "base_link"),
            0.25, rgba=[0, 1, 0, 0.75], marker_ns="eef_goal_pose", marker_id=0)
        vis.visualize_at_config(rrt._goal_config, rgba=[0, 1, 0, 0.25],
                                marker_id=0, marker_ns_prefix="goal_config")
        # Visualize robot and its collision spheres for each node in the reconstructed path.
        if path_counter <= len(path) - 1 and path_counter != 0:
            print(path_counter)
            robot_rgba = [0., 1., 1., 0.5]
            if len(path) - 1 == path_counter:
                robot_rgba = [0., 0., 1., 0.5]
            vis.visualize_at_config(config=path[path_counter], rgba=robot_rgba,
                                    marker_id=100 + path_counter, marker_ns_prefix="path_config")
            print("Visualized " + str(path[path_counter]))
        path_counter += 1

        # Visualize obstacle
        obstacle_rgba = [1., 0, 0, 0.5]
        obstacle_spheres = obstacle_cb.get_spheres('link_1')
        for i in range(len(obstacle_spheres)):
            sphere = obstacle_spheres[i]
            pos = [sphere[0], sphere[1], sphere[2]]
            rad = sphere[3]
            if pos is not None:
                vis.visualize_sphere(pos, rad, obstacle_rgba, "obs_spheres", 10000 + i)

        if smooth_path_counter >= len(smooth_path):
            smooth_path_counter = 0

        # Visualize the current sample.
        vis.visualize_at_config(smooth_path[smooth_path_counter], rgba=[1, 1, 0, 0.25],
                                marker_id=0, marker_ns_prefix="sample_config")
        smooth_path_counter += 1
        rate.sleep()


if __name__ == '__main__':
    main()
