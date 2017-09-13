#!/usr/bin/env python
import rospy
from mashes_tachyon.msg import MsgCalibrate


class Calibrate():
    def __init__(self, parent=None, topic='tachyon/calibrate'):
        self.pub_calibrate = rospy.Publisher(
            topic, MsgCalibrate, queue_size=10)
        self.msg_calibrate = MsgCalibrate()

    def run(self):
        self.msg_calibrate.calibrate = 1
        self.pub_calibrate.publish(self.msg_calibrate)

if __name__ == "__main__":
    rospy.init_node('calibrate_tachyon')

    calibrate = Calibrate()
    calibrate.run()
