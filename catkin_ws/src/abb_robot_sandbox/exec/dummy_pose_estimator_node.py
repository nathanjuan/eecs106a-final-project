#!/usr/bin/env python

import rospy
import sys
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerResponse
import PyKDL


class DummyPoseEstimator:
    def __init__(self):
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._vacuum_gripper_on = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def init(self):
        if not rospy.has_param('~freq'):
            return False
        self._freq = rospy.get_param('~freq')

        if not rospy.has_param('~services/start_vacuum_gripper'):
            return False
        self._start_vacuum_gripper_service = rospy.Service(
            rospy.get_param('~services/start_vacuum_gripper'),
            Trigger, self.start_vacuum_gripper)

        if not rospy.has_param('~services/stop_vacuum_gripper'):
            return False
        self._stop_vacuum_gripper_service = rospy.Service(
            rospy.get_param('~services/stop_vacuum_gripper'),
            Trigger, self.stop_vacuum_gripper)

        # World and robot base frame parameters.
        if not rospy.has_param('~world_frame_id'):
            return False
        self._world_frame_id = rospy.get_param(
            '~world_frame_id')

        if not rospy.has_param('robot_configuration/base_frame_id'):
            return False
        self._robot_base_frame_id = rospy.get_param(
            'robot_configuration/base_frame_id')

        if not rospy.has_param('robot_configuration/tip_frame_id'):
            return False
        self._robot_tip_frame_id = rospy.get_param(
            'robot_configuration/tip_frame_id')

        # Fixed component parameters.
        if not rospy.has_param('~components/fixed/frame_id'):
            return False
        self._fixed_component_frame_id = rospy.get_param(
            '~components/fixed/frame_id')

        if not rospy.has_param('~components/fixed/pose/pos'):
            return False
        self._fixed_pos = rospy.get_param('~components/fixed/pose/pos')

        if not rospy.has_param('~components/fixed/pose/orien'):
            return False
        self._fixed_orien = rospy.get_param('~components/fixed/pose/orien')

        # Moveable component parameters.
        if not rospy.has_param('~components/moveable/frame_id'):
            return False
        self._moveable_component_frame_id = rospy.get_param(
            '~components/moveable/frame_id')

        if not rospy.has_param('~components/moveable/pose/pos'):
            return False
        self._moveable_pos = rospy.get_param('~components/moveable/pose/pos')

        if not rospy.has_param('~components/moveable/pose/orien'):
            return False
        self._moveable_orien = rospy.get_param('~components/moveable/pose/orien')

        return True

    def run(self):
        rate = rospy.Rate(self._freq)

        while not rospy.is_shutdown():
            # Origin of the world frame is colocated with the robot's base frame.
            world_to_base = TransformStamped()
            world_to_base.header.stamp = rospy.Time.now()
            world_to_base.header.frame_id = self._world_frame_id
            world_to_base.child_frame_id = self._robot_base_frame_id
            world_to_base.transform.translation.x = 0
            world_to_base.transform.translation.y = 0
            world_to_base.transform.translation.z = 0
            world_to_base.transform.rotation.x = 0
            world_to_base.transform.rotation.y = 0
            world_to_base.transform.rotation.z = 0
            world_to_base.transform.rotation.w = 1
            self._broadcaster.sendTransform(world_to_base)

            # Broadcast the world to fixed component frame transform.
            world_to_fixed = TransformStamped()
            world_to_fixed.header.stamp = rospy.Time.now()
            world_to_fixed.header.frame_id = self._world_frame_id
            world_to_fixed.child_frame_id = self._fixed_component_frame_id
            world_to_fixed.transform.translation.x = self._fixed_pos[0]
            world_to_fixed.transform.translation.y = self._fixed_pos[1]
            world_to_fixed.transform.translation.z = self._fixed_pos[2]
            if len(self._fixed_orien) == 4:
                # Quaternion.
                world_to_fixed.transform.rotation.x = self._fixed_orien[0]
                world_to_fixed.transform.rotation.y = self._fixed_orien[1]
                world_to_fixed.transform.rotation.z = self._fixed_orien[2]
                world_to_fixed.transform.rotation.w = self._fixed_orien[3]
            elif len(self._fixed_orien) == 3:
                # RPY (Euler).
                orien = quaternion_from_euler(
                    self._fixed_orien[0],
                    self._fixed_orien[1],
                    self._fixed_orien[2])
                world_to_fixed.transform.rotation.x = orien[0]
                world_to_fixed.transform.rotation.y = orien[1]
                world_to_fixed.transform.rotation.z = orien[2]
                world_to_fixed.transform.rotation.w = orien[3]
            else:
                rospy.logerr(
                    "[DummyPoseEstimator] World to fixed rotation invalid!")
                sys.exit(-1)

            self._broadcaster.sendTransform(world_to_fixed)

            if self._vacuum_gripper_on:
                # Get the world to robot tip transform.
                try:
                    world_to_tip_trans = self._tf_buffer.lookup_transform(
                        self._world_frame_id,
                        self._robot_tip_frame_id,
                        rospy.Time())
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rospy.logwarn(
                        "[DummyPoseEstimator] Failed to lookup transform {} to {}".
                        format(self._world_frame_id, self._robot_tip_frame_id))

                world_to_tip = PyKDL.Frame.Identity()
                world_to_tip.p = PyKDL.Vector(
                    world_to_tip_trans.transform.translation.x,
                    world_to_tip_trans.transform.translation.y,
                    world_to_tip_trans.transform.translation.z)
                world_to_tip.M = PyKDL.Rotation.Quaternion(
                    world_to_tip_trans.transform.rotation.x,
                    world_to_tip_trans.transform.rotation.y,
                    world_to_tip_trans.transform.rotation.z,
                    world_to_tip_trans.transform.rotation.w)

                # Update the moveable component's pose, since it's being grasped.
                world_to_moveable = PyKDL.Frame.Identity()
                world_to_moveable.p = PyKDL.Vector(
                    self._moveable_pos[0],
                    self._moveable_pos[1],
                    self._moveable_pos[2])
                if len(self._moveable_orien) == 4:
                    # Quaternion.
                    world_to_moveable.M = PyKDL.Rotation.Quaternion(
                        self._moveable_orien[0],
                        self._moveable_orien[1],
                        self._moveable_orien[2],
                        self._moveable_orien[3])
                elif len(self._moveable_orien) == 3:
                    # RPY (Euler).
                    world_to_moveable.M = PyKDL.Rotation.RPY(
                        self._moveable_orien[0],
                        self._moveable_orien[1],
                        self._moveable_orien[2])
                else:
                    # TODO Throw error
                    pass

                world_to_moveable = world_to_tip * self._tip_to_moveable
                self._moveable_pos[0] = world_to_moveable.p.x()
                self._moveable_pos[1] = world_to_moveable.p.y()
                self._moveable_pos[2] = world_to_moveable.p.z()
                self._moveable_orien = world_to_moveable.M.GetQuaternion()

            # Broadcast the world to moveable component frame transform.
            world_to_moveable = TransformStamped()
            world_to_moveable.header.stamp = rospy.Time.now()
            world_to_moveable.header.frame_id = self._world_frame_id
            world_to_moveable.child_frame_id = self._moveable_component_frame_id
            world_to_moveable.transform.translation.x = self._moveable_pos[0]
            world_to_moveable.transform.translation.y = self._moveable_pos[1]
            world_to_moveable.transform.translation.z = self._moveable_pos[2]
            if len(self._moveable_orien) == 4:
                # Quaternion.
                world_to_moveable.transform.rotation.x = self._moveable_orien[0]
                world_to_moveable.transform.rotation.y = self._moveable_orien[1]
                world_to_moveable.transform.rotation.z = self._moveable_orien[2]
                world_to_moveable.transform.rotation.w = self._moveable_orien[3]
            elif len(self._moveable_orien) == 3:
                # RPY (Euler).
                orien = quaternion_from_euler(
                    self._moveable_orien[0],
                    self._moveable_orien[1],
                    self._moveable_orien[2])
                world_to_moveable.transform.rotation.x = orien[0]
                world_to_moveable.transform.rotation.y = orien[1]
                world_to_moveable.transform.rotation.z = orien[2]
                world_to_moveable.transform.rotation.w = orien[3]
            else:
                rospy.logerr(
                    "[DummyPoseEstimator] World to moveable rotation invalid!")
                sys.exit(-1)

            self._broadcaster.sendTransform(world_to_moveable)

            rate.sleep()

    def start_vacuum_gripper(self, req):
        if self._vacuum_gripper_on:
            rospy.logwarn("[DummyPoseEstimator] Vacuum gripper already on!")
            return TriggerResponse(False, "Vacuum gripper already on")

        try:
            world_to_tip_trans = self._tf_buffer.lookup_transform(
                self._world_frame_id,
                self._robot_tip_frame_id,
                rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(
                "[DummyPoseEstimator] Failed to lookup transform from {} to {}".
                format(self._world_frame_id, self._robot_tip_frame_id))
            return TriggerResponse(False, "Failed to lookup transform")

        world_to_tip = PyKDL.Frame.Identity()
        world_to_tip.p = PyKDL.Vector(
            world_to_tip_trans.transform.translation.x,
            world_to_tip_trans.transform.translation.y,
            world_to_tip_trans.transform.translation.z)
        world_to_tip.M = PyKDL.Rotation.Quaternion(
            world_to_tip_trans.transform.rotation.x,
            world_to_tip_trans.transform.rotation.y,
            world_to_tip_trans.transform.rotation.z,
            world_to_tip_trans.transform.rotation.w)

        world_to_moveable = PyKDL.Frame.Identity()
        world_to_moveable.p = PyKDL.Vector(
            self._moveable_pos[0],
            self._moveable_pos[1],
            self._moveable_pos[2])
        if len(self._moveable_orien) == 4:
            # Quaternion.
            world_to_moveable.M = PyKDL.Rotation.Quaternion(
                self._moveable_orien[0],
                self._moveable_orien[1],
                self._moveable_orien[2],
                self._moveable_orien[3])
        elif len(self._moveable_orien) == 3:
            # RPY (Euler).
            world_to_moveable.M = PyKDL.Rotation.RPY(
                self._moveable_orien[0],
                self._moveable_orien[1],
                self._moveable_orien[2])
        else:
            # TODO Throw error
            pass

        self._tip_to_moveable = world_to_tip.Inverse() * world_to_moveable

        self._vacuum_gripper_on = True
        return TriggerResponse(True, "")

    def stop_vacuum_gripper(self, req):
        success = True
        if not self._vacuum_gripper_on:
            success = False
            rospy.logwarn("[DummyPoseEstimator] Vacuum gripper already off!")

        self._vacuum_gripper_on = False
        return TriggerResponse(success, "")

def main():
    rospy.init_node('dummy_pose_estimator')

    estimator = DummyPoseEstimator()
    if not estimator.init():
        rospy.logerr("[DummyPoseEstimator] Error: failed to initialize!")
        sys.exit(-1)

    estimator.run()


if __name__ == '__main__':
    main()
