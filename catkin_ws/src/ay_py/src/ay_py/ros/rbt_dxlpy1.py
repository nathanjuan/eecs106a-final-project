#! /usr/bin/env python
#\brief   Robot controller for DxlpY1 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.21, 2022
from const import *

from rbt_dxlg import TDxlGripper, TRobotDxlGripper


'''DxlpY1 gripper utility class'''
class TDxlpY1Gripper(TDxlGripper):
  def __init__(self, node_name='gripper_driver', finger_type=None):
    super(TDxlpY1Gripper,self).__init__(node_name=node_name, gripper_type='DxlpY1Gripper', finger_type=finger_type)

  def Cleanup(self):
    super(TDxlpY1Gripper,self).Cleanup()

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q=='DxlpY1Gripper':  return True
    return super(TDxlpY1Gripper,self).Is(q)

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    #WARNING: NotImplemented
    return 0.0


'''Robot control class for DxlpY1Gripper.
  This is defined as a subclass of TMultiArmRobot,
  but actually it does not have a body (only DxlpY1Gripper gripper).
  This virtual body is designed for a compatibility of programs.'''
class TRobotDxlpY1Gripper(TRobotDxlGripper):
  def __init__(self, name='DxlpY1Gripper', gripper_node='gripper_driver', finger_type=None):
    super(TRobotDxlpY1Gripper,self).__init__(name=name,gripper_node=gripper_node)
    self.finger_type= finger_type

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    dxl_gripper= TDxlpY1Gripper(node_name=self.gripper_node, finger_type=self.finger_type)
    return self.internal_init(dxl_gripper)

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('DxlpY1','DxlpY1Gripper'):  return True
    return super(TRobotDxlpY1Gripper,self).Is(q)

