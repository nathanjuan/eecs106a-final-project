#!/usr/bin/python
#\file    rqnb_gui.py
#\brief   RobotiqNB GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.10, 2017
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E,'robotiqnb',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('JoyUSB',':radio',['js0','js1']),
    ('RobotiqNB',[
      (':pair', ('system',['roslaunch ay_util robotiq.launch jsdev:=/dev/input/{JoyUSB}',E]),
                ('kill',['C-c']) )  ]),
    ('Rq-Calib',[
      ('calib',['rosrun ay_util rqnb_calib.py',E])  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('time',[
      (':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                ('kill',['C-c']) )  ]),
    ('aypi10',[
      (':pair', ('stream',['ssh ayg@aypi10 "./stream.sh"',E]),
                ('stop',[E]) ),
      ('config',['ssh ayg@aypi10 "./conf_elp.sh"',E]),
      ('reboot',['ssh ayg@aypi10 "sudo reboot"',E]),
      ('shutdown',['ssh ayg@aypi10 "sudo halt -p"',E])  ]),
    ('fv10',[
      (':pair', ('start',['roslaunch ay_fv_extra fv_pi10.launch',E]),
                ('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start',['roscd ay_trick',E,'rosrun ay_trick direct_run.py "robot \'rqnb\'" j',E]),
                ('quit',['q',E]) )  ]),
    #('aypi3',[
      #(':pair', ('stream',['ssh hm@aypi3 "./stream2.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh hm@aypi3 "sudo reboot"',E])  ]),
    #('monitor1',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8080/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    #('monitor2',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8081/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('RobotiqNB Launcher',widgets,exit_command)
