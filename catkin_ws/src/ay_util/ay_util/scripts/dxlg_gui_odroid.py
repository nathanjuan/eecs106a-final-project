#!/usr/bin/python
#\file    dxlg_gui_odroid.py
#\brief   DxlGripper GUI control panel for demo with Odroid.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.02, 2018
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('JoyUSB',':radio',['js0','js1']),
    ('Joystick',[
      (':pair', ('joy',['rosrun joy joy_node _dev:=/dev/input/{JoyUSB}',E]),
                ('kill',['C-c']) )  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('DxlUSB',':radio',['USB0','USB1']),
    ('DxlGripper',[
      ('test',['rosrun ay_py dxlg1.py /dev/tty{DxlUSB}',E]),
      ('fix_usb',['rosrun ay_util fix_usb_latency.sh tty{DxlUSB}',E])  ]),
    #('rviz',[
      #(':pair', ('rviz',['rviz',E]),
                #('kill',['C-c']) )  ]),
    #('time',[
      #(':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                #('kill',['C-c']) )  ]),
    #('aypi11',[
      #(':pair', ('stream',['ssh aypi11 "./stream.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh aypi11 "sudo reboot"',E]),
      #('shutdown',['ssh aypi11 "sudo halt -p"',E])  ]),
    #('fv11',[
      #(':pair', ('start',['roslaunch ay_fv_extra fv_pi11.launch',E]),
                #('kill',['C-c']) )  ]),
    ('fv11',[
      ('fix_uvc',['rosrun ay_util fix_uvc_video.sh',E]),
      (':pair', ('start',['roslaunch ay_fv_extra fv_odroid.launch',E]),
                ('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start',['rosrun ay_trick direct_run.py "robot \'dxlg\',\'/dev/tty{DxlUSB}\'" j',E]),
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
  RunTerminalTab('DxlGripper Launcher',widgets,exit_command)
