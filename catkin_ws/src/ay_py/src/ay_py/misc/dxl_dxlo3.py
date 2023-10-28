#!/usr/bin/python
#\file    dxl_dxlo3.py
#\brief   Control module of DxlO3 gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.28, 2020
from dxl_util import TDynamixel1
from ..core.util import TRate, CPrint
import time
import threading
import copy

'''DxlO3 gripper utility class'''
class TDxlO3(object):
  def __init__(self, dev='/dev/ttyUSB0'):
    self.dxl_type= ['XH540-W270']*2
    self.dxl_ids= [1,2]
    self.dev= dev
    self.baudrate= 2e6
    self.op_mode= None  #Using default
    self.dxl= [TDynamixel1(dxl_type,dev=self.dev) for dxl_type in self.dxl_type]

    #Thread locker:
    self.port_locker= threading.RLock()
    self.state_locker= threading.RLock()
    self.moveth_locker= threading.RLock()
    self.state= {'stamp':0.0, 'position':[], 'velocity':[], 'effort':[]}
    self.moveth_cmd= {'pos':[],'max_effort':[]}
    self.hz_state_obs= 40  #State observation rate (Hz).
    self.hz_moveth_ctrl= 60  #MoveTh control rate (Hz).
    self.threads= {  #ThreadName:[IsActive,ThreadObject]
      'StateObserver':[False,None],
      'MoveThController':[False,None],}

    #CmdMinMax= [[CmdMin[0],CmdMin[1]],[CmdMax[0],CmdMax[1]]]
    self.CmdMinMax= [
      [2048-370,2048-1110],  #id1-closed,id2-opened
      [2048+1110,2048+370],  #id1-opened,id2-closed
      ]
    self.CmdOrigin=[2048,2048]  #closed,closed

    self.pos_open= [0.6136]*2  #cmd=+400
    self.pos_close= [0.1657]*2  #cmd=+108; With FV+

    #Gripper command-position conversions.
    self.gripper_range= [[-0.5676]*2,[1.7027]*2]  #Position range in radian (lower, upper).
    self.gripper_cmd2pos= lambda cmd: [self.dxl[0].ConvPos(cmd[0]),-self.dxl[1].ConvPos(cmd[1])]
    self.gripper_pos2cmd= lambda pos: [self.dxl[0].InvConvPos(pos[0]),self.dxl[1].InvConvPos(-pos[1])]
    self.gripper_cmd2vel= lambda cmd: [self.dxl[0].ConvVel(cmd[0]),-self.dxl[1].ConvVel(cmd[1])]
    self.gripper_vel2cmd= lambda vel: [self.dxl[0].InvConvVel(vel[0]),self.dxl[1].InvConvVel(-vel[1])]

  '''Initialize (e.g. establish ROS connection).'''
  def Init(self):
    self._is_initialized= False
    res= []
    ra= lambda r: res.append(r)

    with self.port_locker:
      for dxl,id in zip(self.dxl,self.dxl_ids):
        dxl.Baudrate= self.baudrate
        if self.op_mode is not None:  dxl.OpMode= self.op_mode
        dxl.Id= id
        ra(dxl.Setup())

    if all(res):  ra(self.Activate())

    if False not in res:  self._is_initialized= True
    return self._is_initialized

  def Cleanup(self):
    #NOTE: cleaning-up order is important. consider dependency
    if self._is_initialized:
      self.StopMoveTh()
      self.StopStateObs()
      self.Deactivate()
      with self.port_locker:
        for dxl in self.dxl:  dxl.Quit()
      self._is_initialized= False

    #Check the thread lockers status:
    print 'Count of port_locker:',self.port_locker._RLock__count

  '''Range of gripper positions (return: lower, upper).'''
  def PosRange(self):
    return self.gripper_range[0],self.gripper_range[1]

  '''Get current positions (in radian).'''
  def Position(self):
    with self.port_locker:
      pos= [dxl.Position() for dxl in self.dxl]
    if None in pos:
      print 'DxlG: Failed to read position;',pos
      return None
    pos= self.gripper_cmd2pos(pos)
    return pos

  '''Activate gripper (torque is enabled).
    Return success or not.'''
  def Activate(self):
    res= []
    with self.port_locker:
      for dxl in self.dxl:
        res.append(dxl.EnableTorque())
    return all(res)

  '''Deactivate gripper (torque is disabled).
    Return success or not.'''
  def Deactivate(self):
    res= []
    with self.port_locker:
      for dxl in self.dxl:
        res.append(dxl.DisableTorque())
    return all(res)

  '''Open a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Open(self, blocking=False):
    if not self.threads['MoveThController'][0]:
      self.Move(pos=self.pos_open, blocking=blocking)
    else:
      self.MoveTh(pos=self.pos_open, blocking=blocking)

  '''Close a gripper.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Close(self, blocking=False):
    if not self.threads['MoveThController'][0]:
      self.Move(pos=self.pos_close, blocking=blocking)
    else:
      self.MoveTh(pos=self.pos_close, blocking=blocking)

  '''Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def Move(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= [max(cmin,min(cmax,int(c))) for cmin,cmax,c in zip(self.CmdMinMax[0],self.CmdMinMax[1],self.gripper_pos2cmd(pos))]
    max_effort= [max_effort]*len(self.dxl) if isinstance(max_effort,(int,float)) else max_effort
    trg_curr= [dxl.CurrentLimit*me*0.01 for dxl,me in zip(self.dxl,max_effort)]
    with self.port_locker:
      for dxl,c,tc in zip(self.dxl,cmd,trg_curr):
        dxl.MoveToC(c, tc, blocking=False)

    p_log= []  #For detecting stuck.
    while blocking:
      pos= [dxl.Position() for dxl in self.dxl]
      if None in pos:  return
      p_log.append(pos)
      if len(p_log)>50:  p_log.pop(0)
      if all([abs(c-p)<=dxl.GoalThreshold for c,p,dxl in zip(cmd,p_log[-1],self.dxl)]):  break
      #Detecting stuck:
      if len(p_log)>=50 and all([abs(p0-p1)<=dxl.GoalThreshold for p0,p1,dxl in zip(p_log[0],p_log[-1],self.dxl)]):
        print 'DxlG: Control gets stuck. Abort.'
        break


  '''Stop the gripper motion. '''
  def Stop(self, blocking=False):
    if not self.threads['MoveThController'][0]:
      self.Move(self.Position(), blocking=False)
    else:
      self.MoveTh(pos=self.State()['position'], blocking=blocking)


  #Get current state saved in memory (no port access when running this function).
  #Run StartStateObs before using this.
  def State(self):
    with self.state_locker:
      state= copy.deepcopy(self.state)
    return state

  #Start state observation.
  #  callback: Callback function at the end of each observation cycle.
  def StartStateObs(self, callback=None):
    self.StopStateObs()
    th_func= lambda:self.StateObserver(callback)
    self._state_observer_callback= callback  #For future use.
    self.threads['StateObserver']= [True, threading.Thread(name='StateObserver', target=th_func)]
    self.threads['StateObserver'][1].start()

  #Stop state observation.
  def StopStateObs(self):
    if self.threads['StateObserver'][0]:
      self.threads['StateObserver'][0]= False
      self.threads['StateObserver'][1].join()
    self.threads['StateObserver']= [False,None]

  #Set rate (Hz) of state observation.
  #Works anytime.
  def SetStateObsRate(self, rate):
    if self.hz_state_obs!=rate:
      self.hz_state_obs= rate
      if self.threads['StateObserver'][0]:
        self.StartStateObs(self._state_observer_callback)

  '''Thread version of Move: Control a gripper.
    pos: target position in meter.
    max_effort: maximum effort to control; 0 (weakest), 100 (strongest).
    speed: speed of the movement; 0 (minimum), 100 (maximum); NOT_IMPLEMENTED.
    blocking: False: move background, True: wait until motion ends, 'time': wait until tN.  '''
  def MoveTh(self, pos, max_effort=50.0, speed=50.0, blocking=False):
    cmd= [max(cmin,min(cmax,int(c))) for cmin,cmax,c in zip(self.CmdMinMax[0],self.CmdMinMax[1],self.gripper_pos2cmd(pos))]
    with self.moveth_locker:
      self.moveth_cmd= {'pos':pos,'max_effort':[max_effort]*len(self.dxl) if isinstance(max_effort,(int,float)) else max_effort}
    rate= TRate(self.hz_moveth_ctrl)
    p_log= []  #For detecting stuck.
    while blocking:
      p= self.State()['position']
      if None in p:  return
      p_log.append(self.gripper_pos2cmd(p))
      if len(p_log)>50:  p_log.pop(0)
      if all([abs(c-p)<=dxl.GoalThreshold for c,p,dxl in zip(cmd,p_log[-1],self.dxl)]):  break
      #Detecting stuck:
      if len(p_log)>=50 and all([abs(p0-p1)<=dxl.GoalThreshold for p0,p1,dxl in zip(p_log[0],p_log[-1],self.dxl)]):
        CPrint(4,'TDxlO3: Control gets stuck in MoveTh. Abort.')
        break
      #print abs(cmd - p_log[-1]) - self.dxl.GoalThreshold
      rate.sleep()

  #Start MoveTh controller.
  def StartMoveTh(self):
    self.StopMoveTh()
    with self.port_locker:
      goal_pos= self.gripper_cmd2pos([dxl.Read('GOAL_POSITION') for dxl in self.dxl])
      max_effort= [dxl.Read('GOAL_CURRENT')/dxl.CurrentLimit*100.0 for dxl in self.dxl]
    with self.moveth_locker:
      self.moveth_cmd= {'pos':goal_pos,'max_effort':max_effort}
    th_func= lambda:self.MoveThController()
    self.threads['MoveThController']= [True, threading.Thread(name='MoveThController', target=th_func)]
    self.threads['MoveThController'][1].start()

  #Stop MoveTh.
  def StopMoveTh(self):
    if self.threads['MoveThController'][0]:
      self.threads['MoveThController'][0]= False
      self.threads['MoveThController'][1].join()
    self.threads['MoveThController']= [False,None]

  #Set rate (Hz) of MoveTh controller.
  def SetMoveThCtrlRate(self, rate):
    if self.hz_moveth_ctrl!=rate:
      self.hz_moveth_ctrl= rate

  #State observer thread.
  #NOTE: Don't call this function directly.  Use self.StartStateObs and self.State
  def StateObserver(self, callback):
    rate= TRate(self.hz_state_obs)
    while self.threads['StateObserver'][0]:
      with self.port_locker:
        p,v,c= [dxl.Position() for dxl in self.dxl],[dxl.Velocity() for dxl in self.dxl],[dxl.Current() for dxl in self.dxl]
      state= {
        'stamp':time.time(),
        'position':self.gripper_cmd2pos(p) if None not in p else None,
        'velocity':self.gripper_cmd2vel(v) if None not in v else None,
        'effort':[ci/dxl.CurrentLimit*100.0 for ci,dxl in zip(c,self.dxl)] if None not in c else None,
        }
      #print state['position']
      with self.state_locker:
        self.state= state
      if callback is not None:
        callback(state)
      rate.sleep()
    self.threads['StateObserver'][0]= False

  #MoveTh controller thread.
  #NOTE: Don't call this function directly.  Use self.StartMoveTh
  def MoveThController(self):
    rate= TRate(self.hz_moveth_ctrl)
    while self.threads['MoveThController'][0]:
      with self.moveth_locker:
        moveth_cmd= copy.deepcopy(self.moveth_cmd)
      pos,max_effort= moveth_cmd['pos'],moveth_cmd['max_effort']
      cmd= [max(cmin,min(cmax,int(c))) for cmin,cmax,c in zip(self.CmdMinMax[0],self.CmdMinMax[1],self.gripper_pos2cmd(pos))]
      trg_curr= [dxl.CurrentLimit*me*0.01 for dxl,me in zip(self.dxl,max_effort)]

      with self.port_locker:
        for dxl,c,tc in zip(self.dxl,cmd,trg_curr):
          dxl.MoveToC(c, tc, blocking=False)

      #print 'dxl_gripper:MoveThController:rate.remaining:',rate.remaining()
      rate.sleep()
    self.threads['MoveThController'][0]= False

