#!/usr/bin/env python3 
#  

import math
import rospy
import tf2_ros
import tf.transformations
import geometry_msgs 

if __name__ == '__main__':
    
    rospy.init_node("base_to_artag_broadcaster")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    hand_length = 0.3048
    #base endeffector publish + z
    # while not rospy.is_shutdown():
    #     trans = tfBuffer.lookup_transform('base', 'tool0', rospy.Time(), rospy.Duration(10))
    #     trans.transform.translation.z = trans.transform.translation.z - hand_length
    #     trans.header.frame_id = "base"
    #     trans.child_frame_id = "ar_marker_0"
    #     trans.header.stamp = rospy.Time.now()

    #     br.sendTransform(trans)
    #     rate.sleep()
    while not rospy.is_shutdown():
        trans = geometry_msgs.msg.TransformStamped()
        trans.transform.translation.x = 0.167 # 6.25in in meters #artag in 3.25x3.25in
        trans.transform.translation.y = 0.19685 # 7.75 in meters
        q = tf.transformations.quaternion_from_euler(0,0,math.pi/2)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        trans.header.frame_id = "ar_marker_1"
        trans.child_frame_id = "base_link"
        trans.header.stamp = rospy.Time.now()

        br.sendTransform(trans)
        rate.sleep()