import rospy
import math
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from abb_robot_sandbox.cfg import AmplitudeConfig

class Controller(object):
    def __init__(self, freq=15.0):
        self._freq = freq
        self.amp = 1.
        self.vel_pub = rospy.Publisher("joint_vel_cmd", Float64, queue_size=10)
        srv = Server(AmplitudeConfig, self.amp_callback)

    def amp_callback(self, config, level):
        self.amp = config.amplitude
        return config

    def run(self):
        rospy.loginfo('Running controller at {} Hz'.format(self._freq))
        rate = rospy.Rate(self._freq)
        time = 0.
        vel_msg = Float64()
        # while not rospy.is_shutdown():
            # TODO Write controller code here.
        vel_msg.data = math.sin(time) * self.amp
        time += 1 / self._freq
        self.vel_pub.publish(vel_msg)
        rate.sleep()
