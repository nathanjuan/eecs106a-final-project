#! /usr/bin/env python
#Robot controller for Baxter.
from const import *
#if ROS_ROBOT not in ('ANY','Baxter','Baxter_SIM','BaxterN','RobotiqNB'):
  #raise ImportError('Stop importing: ROS_ROBOT is not Baxter')
#if ROS_DISTRO not in ('groovy','hydro','indigo'):  return

import roslib
import rospy
import rospkg
import actionlib
import std_msgs.msg
import sensor_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import baxter_interface
#from baxter_pykdl import baxter_kinematics
import time, math, sys, copy
import cv2
import cv_bridge

from robot import *
from rbt_rq import TRobotiq
from kdl_kin import *

'''Robot control class for Baxter.'''
class TRobotBaxter(TDualArmRobot):
  def __init__(self, name='Baxter', is_sim=False):
    super(TRobotBaxter,self).__init__(name=name)
    self.is_sim= is_sim

    self.joint_names= [[],[]]
    self.joint_names[RIGHT]= ['right_'+joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
    self.joint_names[LEFT]=  ['left_' +joint for joint in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    #Baxter all link names:
    #obtained from /gazebo/link_states
    #obtained from self.planning_scene_req.planning_scene_diff.allowed_collision_matrix.entry_names
    self.links= {}
    self.links['base']= ['base', 'pedestal', 'torso']
    self.links['head']= ['collision_head_link_1', 'collision_head_link_2', 'display', 'head', 'screen', 'sonar_ring']
    self.links['l_arm']= ['left_upper_shoulder', 'left_lower_shoulder', 'left_upper_elbow', 'left_lower_elbow', 'left_upper_forearm', 'left_lower_forearm', 'left_wrist', 'left_upper_elbow_visual', 'left_upper_forearm_visual']
    self.links['l_gripper']= ['left_gripper', 'left_gripper_base', 'left_hand', 'left_hand_accelerometer', 'left_hand_camera', 'left_hand_range']
    self.links['r_arm']= ['right_upper_shoulder', 'right_lower_shoulder', 'right_upper_elbow', 'right_lower_elbow', 'right_upper_forearm', 'right_lower_forearm', 'right_wrist', 'right_upper_elbow_visual', 'right_upper_forearm_visual']
    self.links['r_gripper']= ['right_gripper', 'right_gripper_base', 'right_hand', 'right_hand_accelerometer', 'right_hand_camera', 'right_hand_range']
    self.links['robot']= self.links['base'] + self.links['head'] + self.links['l_arm'] + self.links['l_gripper'] + self.links['r_arm'] + self.links['r_gripper']

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    self.default_face= 'config/baymax.jpg'
    self.ChangeFace(self.default_face)

    self.limbs= [None,None]
    self.limbs[RIGHT]= baxter_interface.Limb(LRTostr(RIGHT))
    self.limbs[LEFT]=  baxter_interface.Limb(LRTostr(LEFT))

    #self.joint_names= [[],[]]
    #self.joint_names[RIGHT]= self.limbs[RIGHT].joint_names()
    #self.joint_names[LEFT]=  self.limbs[LEFT].joint_names()

    #end_link=*_gripper(default),*_wrist,*_hand
    self.kin= [None,None]
    self.kin[RIGHT]= TKinematics(base_link=None,end_link='right_gripper')
    self.kin[LEFT]=  TKinematics(base_link=None,end_link='left_gripper')

    self.head= baxter_interface.Head()

    ra(self.AddActC('r_traj', '/robot/limb/right/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))
    ra(self.AddActC('l_traj', '/robot/limb/left/follow_joint_trajectory',
                    control_msgs.msg.FollowJointTrajectoryAction, time_out=3.0))

    self.robotiq= TRobotiq()  #Robotiq controller
    self.epgripper= TBaxterEPG('right')
    self.grippers= [self.epgripper, self.robotiq]

    print 'Enabling the robot...'
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    print 'Calibrating electric parallel gripper...'
    ra(self.epgripper.Init())

    print 'Initializing and activating Robotiq gripper...'
    ra(self.robotiq.Init())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    for gripper in self.grippers:  gripper.Cleanup()
    super(TRobotBaxter,self).Cleanup()

  '''Configure a state validity checker.'''
  def ConfigureSVC(self, c):
    c.JointNames= copy.deepcopy(self.joint_names)
    c.Links= copy.deepcopy(self.links)
    c.PaddingLinks= c.Links['l_gripper'] + c.Links['r_gripper']
    c.PaddingValues= [0.002]*len(c.PaddingLinks)
    c.DefaultBaseFrame= 'torso'
    c.HandLinkToGrasp[RIGHT]= 'right_gripper'
    c.HandLinkToGrasp[LEFT]= 'left_gripper'
    c.IgnoredLinksInGrasp[RIGHT]= ['right_gripper'] + c.Links['r_gripper']
    c.IgnoredLinksInGrasp[LEFT]= ['left_gripper'] + c.Links['l_gripper']

  '''Answer to a query q by {True,False}. e.g. Is('PR2').'''
  def Is(self, q):
    if q in ('Baxter','Baxter_SIM'):  return True
    return super(TRobotBaxter,self).Is(q)

  @property
  def BaseFrame(self):
    return 'torso'

  '''End link of an arm.'''
  def EndLink(self, arm=None):
    if arm is None:  arm= self.Arm
    if   arm==RIGHT:  return 'right_gripper'
    elif arm==LEFT:   return 'left_gripper'

  '''Names of joints of an arm.'''
  def JointNames(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.joint_names[arm]

  '''Return limits (lower, upper) of joint angles.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def JointLimits(self, arm=None):
    if arm is None:  arm= self.Arm
    return self.kin[arm].joint_limits_lower, self.kin[arm].joint_limits_upper

  '''Return limits of joint angular velocity.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def JointVelLimits(self, arm=None):
    #['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
    return [0.5, 0.5, 0.8, 0.8, 0.8, 0.8, 0.8]

  '''Return range of gripper.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def GripperRange(self, arm=None):
    if arm is None:  arm= self.Arm
    gripper= self.grippers[arm]
    if gripper.Is('BaxterEPG'):  return gripper.PosRange()
    elif gripper.Is('Robotiq'):  return gripper.PosRange()

  '''End effector of an arm.'''
  def EndEff(self, arm):
    if arm is None:  arm= self.Arm
    return self.grippers[arm]

  '''Return joint angles of an arm (list of floats).
    arm: LEFT, RIGHT, or None (==currarm). '''
  def Q(self, arm=None):
    if arm is None:  arm= self.Arm
    with self.sensor_locker:
      angles= self.limbs[arm].joint_angles()
      q= [angles[joint] for joint in self.joint_names[arm]]  #Serialize
    return q

  '''Return joint velocities of an arm (list of floats).
    arm: LEFT, RIGHT, or None (==currarm). '''
  def DQ(self, arm=None):
    if arm is None:  arm= self.Arm
    with self.sensor_locker:
      velocities= self.limbs[arm].joint_velocities()
      dq= [velocities[joint] for joint in self.joint_names[arm]]  #Serialize
    #return dq
    raise NotImplemented('TRobotBaxter.DQ(not tested)')

  '''Compute a forward kinematics of an arm.
  Return self.EndLink(arm) pose on self.BaseFrame.
    return: x, res;  x: pose (list of floats; None if failure), res: FK status.
    arm: LEFT, RIGHT, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned pose is x_ext on self.BaseFrame.
    with_st: whether return FK status. '''
  def FK(self, q=None, x_ext=None, arm=None, with_st=False):
    if arm is None:  arm= self.Arm
    if q is None:  q= self.Q(arm)

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      x= self.kin[arm].forward_position_kinematics(joint_values=angles)

    x_res= list(x if x_ext is None else Transform(x,x_ext))
    return (x_res, True) if with_st else x_res

  '''Compute a Jacobian matrix of an arm.
  Return J of self.EndLink(arm).
    return: J, res;  J: Jacobian (numpy.matrix; None if failure), res: status.
    arm: LEFT, RIGHT, or None (==currarm).
    q: list of joint angles, or None (==self.Q(arm)).
    x_ext: a local pose (i.e. offset) on self.EndLink(arm) frame.
      If not None, we do not consider an offset.
    with_st: whether return the solver status. '''
  def J(self, q=None, x_ext=None, arm=None, with_st=False):
    if arm is None:  arm= self.Arm
    if q is None:  q= self.Q(arm)

    if x_ext is not None:
      #Since KDL does not provide Jacobian computation with an offset x_ext,
      #and converting J with x_ext is not simple, we raise an Exception.
      #TODO: Implement our own FK to solve this issue.
      raise Exception('TRobotBaxter.J: Jacobian with x_ext is not implemented yet.')

    angles= {joint:q[j] for j,joint in enumerate(self.joint_names[arm])}  #Deserialize
    with self.sensor_locker:
      J_res= self.kin[arm].jacobian(joint_values=angles)
    return (J_res, True) if with_st else J_res

  '''Compute an inverse kinematics of an arm.
  Return joint angles for a target self.EndLink(arm) pose on self.BaseFrame.
    return: q, res;  q: joint angles (list of floats; None if failure), res: IK status.
    arm: LEFT, RIGHT, or None (==currarm).
    x_trg: target pose.
    x_ext: a local pose on self.EndLink(arm) frame.
      If not None, the returned q satisfies self.FK(q,x_ext,arm)==x_trg.
    start_angles: initial joint angles for IK solver, or None (==self.Q(arm)).
    with_st: whether return IK status. '''
  def IK(self, x_trg, x_ext=None, start_angles=None, arm=None, with_st=False):
    if arm is None:  arm= self.Arm
    if start_angles is None:  start_angles= self.Q(arm)

    x_trg[3:]/= la.norm(x_trg[3:])  #Normalize the orientation:
    xw_trg= x_trg if x_ext is None else TransformRightInv(x_trg,x_ext)

    with self.sensor_locker:
      res,q= self.kin[arm].inverse_kinematics(xw_trg[:3], xw_trg[3:], seed=start_angles, maxiter=1000, eps=1.0e-6, with_st=True)
    if q is not None:  q= list(q)

    if res:  return (q, True) if with_st else q
    else:  return (q, False) if with_st else None


  '''Follow a joint angle trajectory.
    arm: LEFT, RIGHT, or None (==currarm).
    q_traj: joint angle trajectory [q0,...,qD]*N.
    t_traj: corresponding times in seconds from start [t1,t2,...,tN].
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN. '''
  def FollowQTraj(self, q_traj, t_traj, arm=None, blocking=False):
    assert(len(q_traj)==len(t_traj))
    if arm is None:  arm= self.Arm

    #Insert current position to beginning.
    if t_traj[0]>1.0e-4:
      t_traj.insert(0,0.0)
      q_traj.insert(0,self.Q(arm=arm))

    #copy q_traj, t_traj to goal
    goal= control_msgs.msg.FollowJointTrajectoryGoal()
    goal.goal_time_tolerance= rospy.Time(0.1)
    goal.trajectory.joint_names= self.joint_names[arm]
    goal.trajectory= ToROSTrajectory(self.JointNames(arm), q_traj, t_traj)

    with self.control_locker:
      actc= self.actc.r_traj if arm==RIGHT else self.actc.l_traj
      actc.send_goal(goal)
      BlockAction(actc, blocking=blocking, duration=t_traj[-1])
      #actc.wait_for_result(timeout=rospy.Duration(t_traj[-1]+5.0))  WARNING: Maybe it's better to set timeout


  '''Open a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def OpenGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.1, arm=arm, blocking=blocking)

  '''Close a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    blocking: False: move background, True: wait until motion ends.  '''
  def CloseGripper(self, arm=None, blocking=False):
    self.MoveGripper(pos=0.0, arm=arm, blocking=blocking)

  '''High level interface to control a gripper.
    arm: LEFT, RIGHT, or None (==currarm).
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def MoveGripper(self, pos, max_effort=50.0, speed=50.0, arm=None, blocking=False):
    if self.is_sim:  return  #WARNING:We do nothing if the robot is on simulator.
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    if gripper.Is('BaxterEPG'):
      with self.gripper_locker:
        gripper.Move(pos, max_effort, speed, blocking=blocking)
    elif gripper.Is('Robotiq'):
      with self.gripper_locker:
        gripper.Move(pos, max_effort, speed, blocking=blocking)

  '''Get a gripper position in meter.
    arm: LEFT, RIGHT, or None (==currarm). '''
  def GripperPos(self, arm=None):
    if self.is_sim:  return 0.0  #WARNING:We do nothing if the robot is on simulator.
    if arm is None:  arm= self.Arm

    gripper= self.grippers[arm]
    if gripper.Is('BaxterEPG'):
      with self.sensor_locker:
        pos= gripper.Position()
      return pos
    elif gripper.Is('Robotiq'):
      with self.sensor_locker:
        pos= gripper.Position()
      return pos

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
    NOTE: In the previous versions (before 2019-12-10), this offset was from the opened fingertip position.
      pos: Gripper position to get the offset. None: Current position.
      arm: arm id, or None (==currarm).'''
  def FingertipOffset(self, pos=None, arm=None):
    if arm is None:  arm= self.Arm
    if pos is None:  pos= self.GripperPos(arm)
    return self.grippers[arm].FingertipOffset(pos)

  '''Control the head around z.
    angle: target angle in radian.
    speed: movement speed; 0 (minimum), 100 (maximum).
    timeout: timeout in seconds. '''
  def MoveHeadPan(self, angle, speed=100, timeout=10):
    self.head.set_pan(angle, speed, timeout)

  '''Nod head (no controllable parameters). '''
  def NodHead(self):
    self.head.command_nod()

  '''Change the face image to file_name.'''
  def ChangeFace(self, file_name):
    img= cv2.imread(file_name)
    msg= cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub= rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True, queue_size=1)
    pub.publish(msg)



'''Baxter Electric Parallel Gripper'''
class TBaxterEPG(TGripper2F1):
  def __init__(self, arm='right'):
    super(TBaxterEPG,self).__init__()
    self.epgripper= baxter_interface.Gripper(arm, baxter_interface.CHECK_VERSION)

    #Gripper command-position conversions.
    #epg: electric parallel gripper (narrow, pos#4).
    #self.epg_cmd2pos= lambda cmd: 0.00042*cmd+0.05317    #effective cmd in [0,100] ([0,100])
    #self.epg_pos2cmd= lambda pos: (pos-0.05317)/0.00042  #pos in [0.054,0.094] meter
    #epg: electric parallel gripper (narrow, pos#1).
    self.epg_cmd2pos= lambda cmd: 0.00037*cmd            #effective cmd in [0,100] ([0,100])
    self.epg_pos2cmd= lambda pos: (pos)/0.00037          #pos in [0.00,0.037] meter
    self.epg_range= [0.00,0.037]

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    ra(self.epgripper.calibrate())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  '''Answer to a query q by {True,False}. e.g. Is('Robotiq').'''
  def Is(self, q):
    if q in ('BaxterEPG',):  return True
    return super(TBaxterEPG,self).Is(q)

  '''Range of gripper position.'''
  def PosRange(self):
    return self.epg_range

  '''Get a fingertip height offset in meter.
    The fingertip trajectory of some grippers has a rounded shape.
    This function gives the offset from the highest (longest) point (= closed fingertip position),
    and the offset is always negative.
      pos: Gripper position to get the offset. '''
  def FingertipOffset(self, pos):
    return 0.0

  '''Get current position.'''
  def Position(self):
    return self.epg_cmd2pos(self.epgripper.position())

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    return True  #BaxterEPG is on after calibration.

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    return False  #No way to disable BaxterEPG...?

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    self.Move(100.0, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    self.Move(0.0, blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum).
    blocking: False: move background, True: wait until motion ends.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    clip= lambda c: max(0.0,min(100.0,c))
    cmd= self.epg_pos2cmd(pos)
    self.epgripper.set_velocity(clip(speed))
    self.epgripper.set_moving_force(clip(max_effort))
    self.epgripper.set_holding_force(clip(max_effort))
    self.epgripper.command_position(clip(cmd),block=blocking)

  '''Stop the gripper motion. '''
  def Stop(self, blocking=False):
    pass  #Do nothing.

