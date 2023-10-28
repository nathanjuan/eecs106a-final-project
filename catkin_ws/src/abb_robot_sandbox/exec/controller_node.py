#!/usr/bin/env python

import rospy
from abb_robot_sandbox.controller import Controller


def main():
    rospy.init_node('sim')

    controller = Controller()
    controller.run()


if __name__ == '__main__':
    main()
