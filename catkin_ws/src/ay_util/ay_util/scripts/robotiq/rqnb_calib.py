#!/usr/bin/python
#\file    rqnb_calib.py
#\brief   Setup Robotiq Gripper
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.28, 2017
import roslib; roslib.load_manifest('ay_py')
import rospy
from ay_py.ros.rbt_rq import TRobotiq

if __name__=='__main__':
  rospy.init_node('robotiq_calib')
  rq= TRobotiq()
  print 'Calibrating...'
  rq.Init()
  rospy.sleep(2.0)
  rq.Open(blocking=True)
  print 'Done.'
  rq.Cleanup()
