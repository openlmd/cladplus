#!/usr/bin/env python
import os
import sys
import rospy
import rospkg

from cladplus_control.msg import MsgMode
from cladplus_control.msg import MsgControl
from cladplus_control.msg import MsgPower
from cladplus_control.msg import MsgStep

from mashes_measures.msg import MsgGeometry

from mashes_tachyon.msg import MsgCalibrate

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore


MANUAL = 0
AUTOMATIC = 1
STEP = 2
path = rospkg.RosPack().get_path('mashes_control')


class QtControl(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'control.ui'), self)

        self.btnMode.activated.connect(self.btnModeClicked)
        self.btnControl.clicked.connect(self.btnControlClicked)
        self.btnCalibrate.clicked.connect(self.btnCalibrateClicked)

        self.pub_mode = rospy.Publisher(
            '/control/mode', MsgMode, queue_size=10)
        self.pub_control = rospy.Publisher(
            '/control/parameters', MsgControl, queue_size=10)

        self.pub_calibrate = rospy.Publisher(
            '/tachyon/calibrate', MsgCalibrate, queue_size=10)
        self.pub_step = rospy.Publisher(
            '/control/step', MsgStep, queue_size=10)

        self.mode = MANUAL
        self.msg_mode = MsgMode()
        self.msg_power = MsgPower()
        self.msg_control = MsgControl()
        self.msg_step = MsgStep()

        self.msg_calibrate = MsgCalibrate()

        self.btnControlClicked()

        self.minor_axis = 0
        self.major_axis = 0
        self.power = 0

        rospy.Subscriber(
            '/tachyon/geometry', MsgGeometry, self.cb_geometry, queue_size=1)
        rospy.Subscriber(
            '/control/power', MsgPower, self.cb_power, queue_size=1)

        self.btnModeClicked()
        self.btnControlClicked()

        self.tmrInfo = QtCore.QTimer(self)
        self.tmrInfo.timeout.connect(self.tmrInfoEvent)
        self.tmrInfo.start(100)

    def btnModeClicked(self):
        if self.btnMode.currentText() == "Manual":
            self.lblStatus.setText("Manual")
            self.lblStatus.setStyleSheet(
                "background-color: rgb(255, 255, 0); color: rgb(0, 0, 0);")
            self.mode = MANUAL
            self.tbParams.setCurrentIndex(1)
        elif self.btnMode.currentText() == "Automatic":
            self.lblStatus.setText("Auto")
            self.lblStatus.setStyleSheet(
                "background-color: rgb(0, 0, 255); color: rgb(255, 255, 255);")
            self.mode = AUTOMATIC
            self.tbParams.setCurrentIndex(0)
        elif self.btnMode.currentText() == "Step":
            self.lblStatus.setText("Step")
            self.lblStatus.setStyleSheet(
                "background-color: rgb(0, 255, 0); color: rgb(255, 255, 255);")
            self.mode = STEP
            self.tbParams.setCurrentIndex(2)

        self.msg_mode.value = self.mode
        self.pub_mode.publish(self.msg_mode)

    def btnControlClicked(self):
        self.msg_control.power = self.sbPower.value()
        self.msg_control.setpoint = self.sbWidth.value()
        self.msg_control.kp = self.sbKp.value()
        self.msg_control.ki = self.sbKi.value()
        self.msg_control.kd = self.sbKd.value()
        self.msg_step.trigger = self.sbTime.value()
        self.msg_step.power = self.sbPower_2.value()
        self.pub_control.publish(self.msg_control)
        self.pub_step.publish(self.msg_step)


    def btnCalibrateClicked(self):
        self.msg_calibrate.calibrate = 1
        self.pub_calibrate.publish(self.msg_calibrate)

    def cb_geometry(self, msg_geometry):
        self.minor_axis = msg_geometry.minor_axis
        self.major_axis = msg_geometry.major_axis

    def cb_power(self, msg_power):
        self.power = msg_power.value

    def tmrInfoEvent(self):
        self.lblInfo.setText(
            "Minor axis: %.2f<br>Major axis: %.2f<br><b>Power: %.0f</b>" % (
                self.minor_axis, self.major_axis, self.power))


if __name__ == '__main__':
    rospy.init_node('control_panel')

    app = QtGui.QApplication(sys.argv)
    qt_control = QtControl()
    qt_control.show()
    app.exec_()
