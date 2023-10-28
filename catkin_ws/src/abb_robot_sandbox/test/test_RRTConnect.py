#!/usr/bin/env python

from __future__ import print_function

PKG = 'abb_robot_sandbox'
NAME = 'test_RRTConnect'

import sys
import unittest

import rospy
import rostest

from abb_robot_sandbox.rrt_v2 import RRT, Tree
from abb_robot_sandbox.util import RobotKinematics
import random


class TestRRTConnect(unittest.TestCase):
    def test_sample_config(self):
        rrt = RRT([1, 0, 0], [1, 0, 0], [])
        rrt._start_config = 6 * [0.]
        rrt._goal_config = 6 * [0.1]
        goal_configs_counter = 0
        random_configs_counter = 0
        for i in range(100):
            if rrt.weighted_sample_config("goal") == rrt._goal_config:
                goal_configs_counter += 1
            else:
                random_configs_counter += 1
        print("Random Config Count: {}\nGoal Config Count: {}".format(random_configs_counter, goal_configs_counter))

    def test_get_nearest_neighbour(self):
        rrt = RRT([1, 0, 0], [1, 0, 0], [])
        rrt._start_config = 6 * [0.]
        rrt._goal_config = 6 * [0.1]
        tree = Tree(6*[0.], "goal")
        tree.add_vertex([-0.5] * 6)
        self.assertEquals(tree.get_nearest_neighbour([-1]*6), [-0.5]*6)

    def test_new_config(self):
        rrt = RRT([1, 0, 0], [1, 0, 0], [])
        rrt._start_config = 6 * [0.]
        rrt._goal_config = 6 * [0.1]
        print(rrt.new_config([0.] * 6, [1.0] * 6))

    def test_is_goal(self):
        rrt = RRT([1, 0, 0], [1, 0, 0], [])
        rrt._start_config = 6 * [0.]
        rrt._goal_config = 6 * [0.1]
        self.assertTrue(rrt.is_goal(6 * [0.1], "goal"))
        self.assertTrue(rrt.is_goal(6 * [0.], "start"))
        self.assertFalse(rrt.is_goal(6 * [3.], "goal"))


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRRTConnect, sys.argv)
