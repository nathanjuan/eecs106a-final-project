#!/usr/bin/env python
import rospy
from abb_robot_sandbox.col import CollisionBody
from abb_robot_sandbox.rrt import RRT
from abb_robot_sandbox.vis import Vis


def main():
    rospy.init_node('test_poly')
    vis = Vis()

    # Initialize the RRT.
    robot_cb = CollisionBody()
    rrt = RRT(
        goal_config=[0, 0, 0, 0, 0, 0], seed=44, robot_cb=robot_cb, obstacle_cbs=[])

    # Randomly sample a two configurations.
    start_config = rrt.weighted_sample_config()
    end_config = rrt.weighted_sample_config()
    duration = 1.

    # Fit a polynomial between the two configurations.
    coeffs = []
    for i in range(6):
        ths = rrt.get_coeffs(start_config[i], 0, end_config[i], 0, duration)
        coeffs.append(ths)

    # Take some samples along the interpolated polynomial.
    sampled_configs = []
    dt = 0.01
    t = 0.
    while t < duration:
        sampled_configs.append([rrt.get_values(
            t, coeffs[i][0], coeffs[i][1], coeffs[i][2], coeffs[i][3]) for i in range(6)])
        t += dt

    rate = rospy.Rate(10)
    k = 0
    while not rospy.is_shutdown():
        # Visualize the start configuration.
        vis.visualize_at_config(start_config, rgba=[1, 0, 0, 0.25],
                                marker_id=0, marker_ns_prefix="start_config")

        # Visualize goal position and configuration.
        vis.visualize_at_config(rrt._goal_config, rgba=[0, 1, 0, 0.25],
                                marker_id=0, marker_ns_prefix="goal_config")

        if k >= len(sampled_configs):
            k = 0

        # Visualize the current sample.
        vis.visualize_at_config(sampled_configs[k], rgba=[1, 1, 0, 0.25],
                                marker_id=0, marker_ns_prefix="sample_config")
        k += 1
        rate.sleep()


if __name__ == '__main__':
    main()
