#!/usr/bin/env python
import os
import rospy
import rospkg

from cladplus_control.msg import MsgMode
from cladplus_control.msg import MsgControl
from cladplus_control.msg import MsgPower
from cladplus_control.msg import MsgStart
from mashes_measures.msg import MsgGeometry
from mashes_measures.msg import MsgStatus

from control.control import Control
from control.control import PID
import numpy as np


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
            '/control/parameters', MsgControl, self.cb_control, queue_size=1)
        rospy.Subscriber(
            '/supervisor/status', MsgStatus, self.cb_status, queue_size=1)
        self.pub_power = rospy.Publisher(
            '/control/power', MsgPower, queue_size=10)
        self.pub_start = rospy.Publisher(
            '/control/start', MsgStart, queue_size=10)


        self.msg_power = MsgPower()
        self.msg_start = MsgStart()
        self.mode = MANUAL

        self.status = False
        self.time_step = float(0)
        self.track_number = 0
        self.step_increase = 100
        self.control = Control()
        self.updateParameters()
        self.time_control = 0
        self.start = False

        self.t_reg = 1
        #En caso de un tubo de 4 cm de diametro deseable que tome de referencia 3 cordon.
        #self.t_reg = 16
        # self.t_auto = 40

        self.track = []

        self.setPowerParameters(rospy.get_param('/control/power'))
        self.control.pid.set_limits(self.power_min, self.power_max)
        self.control.pid.set_setpoint(self.setpoint)

        rospy.spin()

    def setParameters(self, params):
        self.Kp = params['Kp']
        self.Ki = params['Ki']
        self.Kd = params['Kd']
        self.control.pid.set_parameters(self.Kp, self.Ki, self.Kd)
        # print 'Kp:', self.Kp, 'Ki:', self.Ki, 'Kd', self.Kd

    def setManualParameters(self, params):
        self.power = params['power']

    def setAutoParameters(self, params):
        self.setpoint = params['width']
        self.t_auto = params['time']
        self.track_control = params['track_number']

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
        self.track_number = 0
        rospy.loginfo('Mode: ' + str(self.mode))

    def cb_control(self, msg_control):
        self.updateParameters()
        self.track_number = 0
        self.control.pid.set_setpoint(self.setpoint)

    def cb_status(self, msg_status):
        self.power_ant = msg_status.power
        if msg_status.laser_on and not self.status:
                self.time_step = 0
                self.track_number += 1
        self.status = msg_status.laser_on

    def cb_geometry(self, msg_geo):
        stamp = msg_geo.header.stamp
        time = stamp.to_sec()
        if self.mode == MANUAL:
            value = self.manual(self.power)
        elif self.mode == AUTOMATIC:
            value = self.automatic(msg_geo.minor_axis, time)
        elif self.mode == STEP:
            value = self.step(time)
        # value = self.cooling(msg_geo.minor_axis, value)
        value = self.range(value)
        self.msg_power.header.stamp = stamp
        self.msg_power.value = value
        self.msg_power.time = str(self.time_control)
        self.msg_power.track_number = self.track_number
        rospy.set_param('/process/power', value)
        self.pub_power.publish(self.msg_power)
        start = self.msg_start.control
        if start is not self.start:
            self.msg_start.setpoint = self.setpoint
            self.pub_start.publish(self.msg_start)
        self.start = start


    def manual(self, power):
        value = self.control.pid.power(power)
        return value

    def automatic(self, minor_axis, time):
        if self.time_step == 0:
            self.time_step = time
        self.time_control= time - self.time_step
        #Funcionamiento varios cordones.
        #taking control parameters
        # if minor_axis > 0.5 and self.track_number is 3:
        #     print 'taking reference data'
        #     self.track.append(minor_axis)
        #     self.setpoint = sum(self.track)/len(self.track)
        #     auto = {'width': self.setpoint}
        #     rospy.set_param('/control/automatic', auto)
        #     self.control.pid.set_setpoint(self.setpoint)

        # condicion inicio control
        # if minor_axis > 0.5 and self.track_number >= 4:
        #     value = self.control.pid.update(minor_axis, time)
        # else:
        #     value = self.control.pid.power(self.power)

        #funcionamiento en continuo, inicia a registrar en t_reg e inicia control en t_auto
        if self.status and self.time_step > 0 and self.time_control > self.t_reg and self.time_control < self.t_auto:
            self.track.append(minor_axis)
            self.setpoint = sum(self.track)/len(self.track)
            self.control.pid.set_setpoint(self.setpoint)
        if self.status and self.time_step > 0 and self.time_control > self.t_auto:
            value = self.control.pid.update(minor_axis, time)
            self.msg_start.control = True
        else:
            value = self.power
            self.msg_start.control = False
        return value

    def step(self, time):
        #Para programalo para un tempo de salto
        if self.time_step == 0:
            self.time_step = time
        if self.status and self.time_step > 0 and time - self.time_step > self.trigger:
            value = self.power_step
        else:
            value = self.power
        #Para saltar de potencia en potencia
        # if self.track_number > 1:
        #     value = self.power_step - ((self.track_number-1)*self.step_increase)
        # else:
        #     value = self.power_step
        # value = self.range(value)
        #Para un valor fijo no segundo cordon
        # if self.track_number > 1:
        #     value = self.power_step
        # else:
        #     value = self.power
        # value = self.range(value)
        return value

    def range(self, value):
        if value < self.power_min:
            value = self.power_min
        elif value > self.power_max:
            value = self.power_max
        return value

    def cooling(self, msg_geo, value):
        #condicion de parada
        if msg_geo > 4.5:
            value = 200
        return value
        #funcion para que non se sobrequente
        #sperar a que enfrie ou parar


if __name__ == '__main__':
    try:
        NdControl()
    except rospy.ROSInterruptException:
        pass
