#!/usr/bin/python
#\file    bx_off_bodyavd.py
#\brief   Disable self collision avoidance
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1

import roslib
import rospy
import std_msgs.msg
import time

if __name__=='__main__':
  rospy.init_node('bx_off_bodyavd')
  pub= [None,None]
  pub[0]= rospy.Publisher('/robot/limb/right/suppress_collision_avoidance', std_msgs.msg.Empty, latch=False)
  pub[1]= rospy.Publisher('/robot/limb/left/suppress_collision_avoidance', std_msgs.msg.Empty, latch=False)

  empty_msg= std_msgs.msg.Empty()
  r= rospy.Rate(10)
  while not rospy.is_shutdown():
    pub[0].publish(empty_msg)
    pub[1].publish(empty_msg)
    r.sleep()
  #rospy.spin()
