import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Sim(object):
    def __init__(self, freq=15.0):
        self._freq = freq

        # TODO Add publishers / subscribers here.
        self.vel = 0.
        self.vel_sub = rospy.Subscriber('joint_vel_cmd', Float64, self.vel_callback)
        self._joint_states_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=10)

    def vel_callback(self, data):
        self.vel = data.data

    def run(self):
        rospy.loginfo('Running simulator at {} Hz'.format(self._freq))
        rate = rospy.Rate(self._freq)

        # This is where we will maintain the current joint angles (or positions)
        # of the robot. On each iteration of the simulation, we will update the
        # joint positions and re-publish this message.
        msg = JointState()
        msg.name = ['joint_{}'.format(i+1) for i in range(6)]
        msg.position = [0.] * 6
        msg.velocity = [0.] * 6
        # while not rospy.is_shutdown():
            # Simulation code
        for i in range(6):
            msg.position[i] += 1/self._freq * self.vel

            # Publish the current joint positions.
            msg.header.stamp = rospy.Time.now()
            self._joint_states_pub.publish(msg)

            rate.sleep()
