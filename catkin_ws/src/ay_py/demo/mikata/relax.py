#!/usr/bin/python
#Relax Dynamixel servos (set all PWMs to zero).

from _path import *
from ay_py.misc.dxl_mikata import *

#Setup the device
mikata= TMikata()
mikata.Setup()
mikata.EnableTorque()

mikata.SetPWM({jname:0 for jname in mikata.JointNames()})

#mikata.DisableTorque()
mikata.Quit()
