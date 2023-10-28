#!/usr/bin/python
#\file    bx_gui.py
#\brief   Baxter GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.01, 2017
#\date    Apr.14, 2017
#\date    Sep.10, 2017
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E,'bxmaster',E)),
      ('Exit',':close') ]),
    ('Baxter',[
      (':pair', ('enable',['bxrun -e',E]),
                ('disable',['bxrun -d',E]) ),
      (':pair', ('system',['roslaunch ay_util bx_real.launch',E]),
                ('kill',['C-c']) )  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('time',[
      (':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                ('kill',['C-c']) )  ]),
    ('m100',[
      (':pair', ('start',['roslaunch ay_3dvision sentis_tof_m100_s.launch',E]),
                ('kill',['C-c']) )  ]),
    ('pose_est',[
      (':pair', ('start',['roslaunch ay_3dvision rt_pose_estimator_m100.launch',E]),
                ('kill',['C-c']) )  ]),
    ('segm_obj',[
      (':pair', ('start',['roslaunch ay_vision segm_obj.launch',E]),
                ('kill',['C-c']) )  ]),
    ('aypi1-R',[
      (':pair', ('stream',['ssh hm@aypi1 "./stream.sh"',E]),
                ('stop',[E]) ),
      ('config',['ssh hm@aypi1 "./conf_elp.sh"',E]),
      ('reboot',['ssh hm@aypi1 "sudo reboot"',E]),
      ('shutdown',['ssh hm@aypi1 "sudo halt -p"',E])  ]),
    ('vskin1-R',[
      (':pair', ('start',['roslaunch ay_fv_extra fv_pi01.launch',E]),
                ('kill',['C-c']) )  ]),
    ('aypi2-L',[
      (':pair', ('stream',['ssh hm@aypi2 "./stream.sh"',E]),
                ('stop',[E]) ),
      ('config',['ssh hm@aypi2 "./conf_elp.sh"',E]),
      ('reboot',['ssh hm@aypi2 "sudo reboot"',E]),
      ('shutdown',['ssh hm@aypi2 "sudo halt -p"',E])  ]),
    ('vskin2-L',[
      (':pair', ('start',['roslaunch ay_fv_extra fv_pi02.launch',E]),
                ('kill',['C-c']) )  ]),
    ('aypi3',[
      (':pair', ('stream',['ssh hm@aypi3 "./stream2.sh"',E]),
                ('stop',[E]) ),
      ('config',['ssh hm@aypi3 "./conf_elp.sh"',E]),
      ('reboot',['ssh hm@aypi3 "sudo reboot"',E]),
      ('shutdown',['ssh hm@aypi3 "sudo halt -p"',E])  ]),
    ('monitor1',[
      (':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8080/?action=stream&dummy=file.mjpg"',E]),
                ('kill',['C-c']) )  ]),
    ('monitor2',[
      (':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8081/?action=stream&dummy=file.mjpg"',E]),
                ('kill',['C-c']) )  ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('Baxter Launcher',widgets,exit_command)
