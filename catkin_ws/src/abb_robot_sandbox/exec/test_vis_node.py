#!/usr/bin/env python
import rospy
from abb_robot_sandbox.vis import Vis


def main():
    rospy.init_node('test_vis')
    vis = Vis()

    config_seq = [[0, 0, 0, 0, 0, 0],
                  [0.5, 0.5, 0, 0, 0, 0],
                  [0.5, 1.0, 0, 0.5, 0, 0.5],
                  [1.2, 1.0, 0, 0.5, 0, 1.0]]

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        for config in config_seq:
            rgba = [0.8, 0.8, 0.8, 0.5]
            vis.visualize_at_config(config, rgba=rgba, marker_id=0)
            rate.sleep()


if __name__ == '__main__':
    main()
