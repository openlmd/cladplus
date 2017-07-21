#!/usr/bin/env python
import rospy

from cladplus_control.msg import MsgPower

from analog.analog import Analog
# from analog.analog_12 import Analog_12 as Analog


class NdAnalog():
    def __init__(self):
        rospy.init_node('analog')

        power_min = rospy.get_param('~power_min', 0)
        power_max = rospy.get_param('~power_max', 1500)
        laser_percent = rospy.get_param('~laser_percent', 100)

        self.analog = Analog()
        self.analog.power_factor(power_min, power_max)
        self.line_power(laser_percent)

        rospy.Subscriber("/control/power", MsgPower, self.cb_power)
        rospy.spin()

    def cb_power(self, msg_power):
        self.analog.reg = 5000
        output = self.analog.factor * msg_power.value
        rospy.loginfo("Power: %.2f, Output: %.2f", msg_power.value, output)
        self.analog.output(output)

    def line_power(self, laser_percent):
        self.analog.reg = 5002
        output = 5 - 0.05 * laser_percent
        rospy.loginfo("Line power: %i", output)
        self.analog.output(output)


if __name__ == '__main__':
    try:
        NdAnalog()
    except rospy.ROSInterruptException:
        pass
