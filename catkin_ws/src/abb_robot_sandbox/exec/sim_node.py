#!/usr/bin/env python

import rospy
from abb_robot_sandbox.sim import Sim


def main():
    rospy.init_node('sim')

    sim = Sim()
    sim.run()


if __name__ == '__main__':
    main()
