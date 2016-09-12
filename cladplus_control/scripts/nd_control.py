#!/usr/bin/env python
import os
import rospy
import rospkg

from cladplus_control.msg import MsgMode
from cladplus_control.msg import MsgControl
from cladplus_control.msg import MsgPower
from mashes_measures.msg import MsgGeometry
from mashes_measures.msg import MsgStatus

from control.control import Control
from control.control import PID

MANUAL = 0
STEP = 2
AUTOMATIC = 1
RAMP = 3

class NdControl():
    def __init__(self):
        rospy.init_node('control')

        rospy.Subscriber(
            '/tachyon/geometry', MsgGeometry, self.cb_geometry, queue_size=1)
        rospy.Subscriber(
            '/control/mode', MsgMode, self.cb_mode, queue_size=1)
        rospy.Subscriber(
            '/control/parameters', MsgControl, self.cb_control, queue_size=1)
        rospy.Subscriber(
            '/supervisor/status', MsgStatus, self.cb_status, queue_size=1)

        self.pub_power = rospy.Publisher(
            '/control/power', MsgPower, queue_size=10)

        self.msg_power = MsgPower()
        self.mode = MANUAL

        self.status = False
        self.time_step = None
        self.control = Control()
        self.updateParameters()

        self.ramp_time = 0
        self.ramp_step = 100
        self.ramp_max = 1500
        self.ramp_min = 500
        self.ramp_count = 0


        self.setPowerParameters(rospy.get_param('/control/power'))
        self.control.pid.set_limits(self.power_min, self.power_max)
        self.control.pid.set_setpoint(self.setpoint)

        rospy.spin()

    def setParameters(self, params):
        self.Kp = params['Kp']
        self.Ki = params['Ki']
        self.Kd = params['Kd']
        self.control.pid.set_parameters(self.Kp, self.Ki, self.Kd)
        print 'Kp:', self.Kp, 'Ki:', self.Ki, 'Kd', self.Kd

    def setManualParameters(self, params):
        self.power = params['power']

    def setAutoParameters(self, params):
        self.setpoint = params['width']

    def setPowerParameters(self, params):
        self.power_min = params['min']
        self.power_max = params['max']

    def setStepParameters(self, params):
        self.power_step = params['power']
        self.trigger = params['trigger']

    def updateParameters(self):
        self.setParameters(rospy.get_param('/control/parameters'))
        self.setStepParameters(rospy.get_param('/control/step'))
        self.setManualParameters(rospy.get_param('/control/manual'))
        self.setAutoParameters(rospy.get_param('/control/automatic'))

    def cb_mode(self, msg_mode):
        self.mode = msg_mode.value
        rospy.loginfo('Mode: ' + str(self.mode))

    def cb_control(self, msg_control):
        self.updateParameters()
        rospy.loginfo(rospy.get_param('/control/manual'))
        rospy.loginfo(rospy.get_param('/control/step'))
        self.control.pid.set_setpoint(self.setpoint)

    def cb_status(self, msg_status):
        #print msg_status.laser_on, self.status, self.time_step
        if msg_status.laser_on and not self.status:
             self.time_step = 0
             self.ramp_time = 0
	self.status = msg_status.laser_on

    def cb_geometry(self, msg_geo):
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
        elif self.mode == RAMP:
            if self.ramp_time == 0:
                self.ramp_time = time
                self.ramp_count = self.ramp_min
            if self.status and time - self.ramp_time > 1  and self.ramp_count < self.ramp_max:
                self.ramp_time = time
                self.ramp_count = self.ramp_count + self.ramp_step
                print self.ramp_count
            value = self.ramp_count

        elif self.mode == STEP:
	    if self.time_step == 0:
                self.time_step = time
            if self.status and self.time_step > 0 and time - self.time_step > self.trigger:
                value = self.power_step
            else:
                value = self.power

        else:
            major_axis = msg_geo.major_axis
            if major_axis:
                value = self.control.pid.update(major_axis, time)
            else:
                value = self.control.pid.power(self.power)
        self.msg_power.header.stamp = stamp
        self.msg_power.value = value
        #print '# Timestamp', time, '# Power', self.msg_power.value, self.time_step
        self.pub_power.publish(self.msg_power)

if __name__ == '__main__':
    try:
        NdControl()
    except rospy.ROSInterruptException:
        pass
