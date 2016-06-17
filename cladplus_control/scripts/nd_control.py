#!/usr/bin/env python
import os
import rospy
import rospkg

#from std_msgs.msg import String
from cladplus_control.msg import MsgMode
from cladplus_control.msg import MsgStep
from cladplus_control.msg import MsgControl
from cladplus_control.msg import MsgPower
from cladplus_labjack.msg import MsgLabJack
from mashes_measures.msg import MsgGeometry

from control.control import Control
from control.control import PID


MANUAL = 0
STEP = 2
AUTOMATIC = 1


class NdControl():
    def __init__(self):
        rospy.init_node('control')

        rospy.Subscriber(
            '/tachyon/geometry', MsgGeometry, self.cb_geometry, queue_size=1)
        rospy.Subscriber(
            '/control/mode', MsgMode, self.cb_mode, queue_size=1)
        rospy.Subscriber(
            '/control/step', MsgStep, self.cb_step, queue_size=1)
        rospy.Subscriber(
            '/control/parameters', MsgControl, self.cb_control, queue_size=1)

        self.pub_power = rospy.Publisher(
            '/control/power', MsgPower, queue_size=10)

        power_min = rospy.get_param('~power_min', 0.0)
        power_max = rospy.get_param('~power_max', 1500.0)

        power = rospy.get_param('~power', 1000)
        setpoint = rospy.get_param('~setpoint', 3.0)
        Kp = rospy.get_param('~Kp', 5.0)
        Ki = rospy.get_param('~Ki', 100.0)
        Kd = rospy.get_param('~Kd', 0.0)
        print 'Kp:', Kp, 'Ki:', Ki, 'Kd', Kd

        self.msg_power = MsgPower()
        self.msg_labjack = MsgLabJack()

        self.mode = MANUAL
        self.power = power
        self.setpoint = setpoint
        self.secs = 5
        self.first_step = True

        self.control = Control()
        path = rospkg.RosPack().get_path('cladplus_control')
        self.control.load_conf(os.path.join(path, 'config/control.yaml'))
        self.control.pid.set_limits(power_min, power_max)
        self.control.pid.set_setpoint(self.setpoint)

        rospy.spin()

    def cb_mode(self, msg_mode):
        self.mode = msg_mode.value
        rospy.loginfo('Mode: ' + str(self.mode))
        if self.mode == 2:
            self.first_step = True

    def cb_step(self, msg_step):
        self.secs = msg_step.trigger
        self.power_step = msg_step.power
        rospy.loginfo('Step params: ' + str(msg_step))

    def cb_control(self, msg_control):
        self.setpoint = msg_control.setpoint
        Kp = msg_control.kp
        Ki = msg_control.ki
        Kd = msg_control.kd

        self.control.pid.set_setpoint(self.setpoint)
        self.control.pid.set_parameters(Kp, Ki, Kd)
        rospy.loginfo('Params: ' + str(msg_control))

    def cb_geometry(self, msg_geo):
        print 'geometry'
        stamp = msg_geo.header.stamp
        time = stamp.to_sec()
        if self.mode == MANUAL:
            value = self.control.pid.power(self.power)
        elif self.mode == AUTOMATIC:
            minor_axis = msg_geo.minor_axis
            if minor_axis > 0.5:
                value = self.control.pid.update(minor_axis, time)
            else:
                value = self.control.pid.power(self.power)
        elif self.mode == STEP:
            if self.first_step:
                self.time_step = stamp.to_sec()
                self.first_step = False
            if time - self.time_step > self.secs:
                value = self.power_step
            else:
                value = 0
        else:
            major_axis = msg_geo.major_axis
            if major_axis:
                value = self.control.pid.update(major_axis, time)
            else:
                value = self.control.pid.power(self.power)
        self.msg_power.header.stamp = stamp
        self.msg_power.value = value
        print '# Timestamp', time, '# Power', self.msg_power.value
        self.pub_power.publish(self.msg_power)


if __name__ == '__main__':
    try:
        NdControl()
    except rospy.ROSInterruptException:
        pass
