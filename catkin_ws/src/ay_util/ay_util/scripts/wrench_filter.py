#!/usr/bin/python
#\file    wrench_filter.py
#\brief   Apply a moving average filter to a wrench topic.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.19, 2021
import roslib
import rospy
roslib.load_manifest('ay_py')
from ay_py.core import TContainer
from ay_py.ros import XYZToVec, VecToXYZ
roslib.load_manifest('geometry_msgs')
import geometry_msgs.msg
import numpy as np
import copy

def CallbackWrench(msg, status):
  status.wrench_seq.append(XYZToVec(msg.wrench.force)+XYZToVec(msg.wrench.torque))
  if len(status.wrench_seq)>status.filter_len:
    status.wrench_seq.pop(0)
  wrench_filtered= np.mean(status.wrench_seq,axis=0)
  msg_out= copy.deepcopy(msg)
  VecToXYZ(wrench_filtered[:3], msg_out.wrench.force)
  VecToXYZ(wrench_filtered[3:], msg_out.wrench.torque)
  status.pub_wrench.publish(msg_out)

if __name__=='__main__':
  rospy.init_node('wrench_filter')
  topic_in= rospy.get_param('~topic_in', '/wrench')
  topic_out= rospy.get_param('~topic_out', '/wrench_filtered')
  filter_len= rospy.get_param('~N', 10)

  #Filtered wrench:
  pub_wrench= rospy.Publisher(topic_out, geometry_msgs.msg.WrenchStamped, queue_size=10)

  status= TContainer()
  status.filter_len= filter_len
  status.wrench_seq= []
  status.pub_wrench= pub_wrench
  sub_wrench_in= rospy.Subscriber(topic_in, geometry_msgs.msg.WrenchStamped, lambda msg,status=status:CallbackWrench(msg,status))

  rospy.spin()
