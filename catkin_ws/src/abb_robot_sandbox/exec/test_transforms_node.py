#!/usr/bin/env python
import rospy
import os
from rospkg import RosPack
from abb_robot_sandbox.util import TransformManager, RobotKinematics
from abb_robot_sandbox.vis import Vis
from abb_robot_sandbox.col import CollisionBody


def main():
    rospy.init_node('test_transforms')
    tm = TransformManager()
    vis = Vis()
    kin = RobotKinematics()

    rate = rospy.Rate(1.0)

    # Load the robot's collision body from the YAML config file.
    cb = CollisionBody()
    rp = RosPack()
    path = rp.get_path('abb_robot_sandbox')
    cb.load(os.path.join(path, 'config/robot_collision_spheres.yaml'))
    print(cb._frame_to_spheres)

    robot_config = [0.5, 0.5, 0, 0, 0, 0]
    robot_vel = [0, 0.01, 0, 0.01, 0.01, 0]
    # spheres = [[0.15, 0, 0, 0.1],
    #            [0.3, 0, 0, 0.1]]
    # sphere_frame = 'link_4'

    while not rospy.is_shutdown():
        robot_rgba = [0.8, 0.8, 0.8, 0.25]
        sphere_rgba = [1, 0, 0, 0.75]
        vis.visualize_at_config(robot_config, rgba=robot_rgba, marker_id=0)

        fk_pos = kin.fk_pos(robot_config)
        ik_jnts = kin.ik(fk_pos)
        print("#####")
        print("** FK: {}".format(fk_pos))
        print("** IK: {}".format(ik_jnts))
        print("** Robot config: {}".format(robot_config))
        print("** FK(IK): {}".format(kin.fk_pos(ik_jnts)))
        print("** FK(Robot config): {}".format(kin.fk_pos(robot_config)))

        # Move the robot's joints.
        for i in range(len(robot_config)):
            robot_config[i] += robot_vel[i]

        rate.sleep()


if __name__ == '__main__':
    main()
