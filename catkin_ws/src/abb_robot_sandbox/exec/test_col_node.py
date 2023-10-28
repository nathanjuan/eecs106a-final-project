#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
import std_msgs
from abb_robot_sandbox.col import col_handler

def main():
    rospy.init_node('collision_test_markers')
    vis_pub = rospy.Publisher('collision_test_markers', Marker, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg1 = Marker()
        msg1.header.frame_id = 'map'

        msg1.ns = 'collision_test_markers'
        msg1.id = 0

        msg1.pose.position.x = 0.5
        msg1.pose.position.y = 0
        msg1.pose.position.z = 0.5

        msg1.pose.orientation.w = 1

        msg1.type = Marker.SPHERE
        msg1.scale.x = 1.
        msg1.scale.y = 1.
        msg1.scale.z = 1.

        msg1.color.r = 0.
        msg1.color.g = 1.
        msg1.color.b = 0.
        msg1.color.a = 0.3

        msg1.lifetime = rospy.Duration(1)

        sphere_1 = [msg1.pose.position.x, msg1.pose.position.y, msg1.pose.position.z, msg1.scale.x/2]


        msg2 = Marker()
        msg2.header.frame_id = 'map'

        msg2.ns = 'collision_test_markers'
        msg2.id = 1

        msg2.pose.position.x = 1.75
        msg2.pose.position.y = 0
        msg2.pose.position.z = 0.5

        msg2.pose.orientation.w = 1

        msg2.type = Marker.SPHERE
        msg2.scale.x = 1.
        msg2.scale.y = 1.
        msg2.scale.z = 1.

        msg2.color.r = 0.
        msg2.color.g = 1.
        msg2.color.b = 0.
        msg2.color.a = 0.3
        
        msg2.lifetime = rospy.Duration(1)
        
        sphere_2 = [msg2.pose.position.x, msg2.pose.position.y, msg2.pose.position.z, msg2.scale.x/2]
        
        col = col_handler()
        if col.col(sphere_1, sphere_2):
            msg1.color.r = 1.
            msg1.color.g = 0.
            msg2.color.r = 1.
            msg2.color.g = 0.
        else:
            msg1.color.r = 0.
            msg1.color.g = 1.
            msg2.color.r = 0.
            msg2.color.g = 1.
        
        vis_pub.publish(msg1)
        vis_pub.publish(msg2)
        rate.sleep()

if __name__ == "__main__":
    main()
