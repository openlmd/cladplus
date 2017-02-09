#!/usr/bin/env python
import os
import sys
import rospy
import rospkg

# from cladplus_control.msg import MsgMode
from cladplus_control.msg import MsgControl
from cladplus_control.msg import MsgPower
from cladplus_control.msg import MsgInfo
from cladplus_control.msg import MsgStart

from mashes_tachyon.msg import MsgCalibrate
from mashes_measures.msg import MsgGeometry

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore


MANUAL = 0
AUTOMATIC = 1
STEP = 2
path = rospkg.RosPack().get_path('cladplus_control')


class QtControl(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        loadUi(os.path.join(path, 'resources', 'control.ui'), self)

        self.btnMode.activated.connect(self.btnModeClicked)
        self.btnAuto.activated.connect(self.btnAutoClicked)
        self.btnControl.clicked.connect(self.btnControlClicked)
        self.btnCalibrate.clicked.connect(self.btnCalibrateClicked)

        # self.pub_mode = rospy.Publisher(
        #     '/control/mode', MsgMode, queue_size=10)
        self.pub_control = rospy.Publisher(
            '/control/parameters', MsgControl, queue_size=10)
        self.pub_calibrate = rospy.Publisher(
            '/tachyon/calibrate', MsgCalibrate, queue_size=10)

        self.mode = MANUAL
        # self.msg_mode = MsgMode()
        self.msg_control = MsgControl()
        self.msg_calibrate = MsgCalibrate()

        self.minor_axis = 0
        self.major_axis = 0
        self.power = 0
        self.control = False

        self.track_number= 0
        self.time_control = 0

        self.auto_mode = 0


        rospy.Subscriber(
            '/tachyon/geometry', MsgGeometry, self.cb_geometry, queue_size=1)
        rospy.Subscriber(
            '/control/power', MsgPower, self.cb_power, queue_size=1)
        rospy.Subscriber(
            '/control/info', MsgInfo, self.cb_info, queue_size=1)
        rospy.Subscriber(
            '/control/start', MsgStart, self.cb_start, queue_size=1)

        self.btnModeClicked()

        self.tmrInfo = QtCore.QTimer(self)
        self.tmrInfo.timeout.connect(self.tmrInfoEvent)
        self.tmrInfo.start(500)

        self.setParameters(rospy.get_param('/control/parameters'))
        self.setStepParameters(rospy.get_param('/control/step'))
        self.setManualParameters(rospy.get_param('/control/manual'))
        self.setAutoParameters(rospy.get_param('/control/automatic'))
        self.btnControlClicked()

    def setParameters(self, params):
        self.sbKp.setValue(params['Kp'])
        self.sbKi.setValue(params['Ki'])
        self.sbKd.setValue(params['Kd'])

    def getParameters(self):
        params = {'Kd': self.sbKd.value(),
                  'Ki': self.sbKi.value(),
                  'Kp': self.sbKp.value()}
        return params

    def setStepParameters(self, params):
        self.sbPower_2.setValue(params['power'])
        self.sbTime.setValue(params['trigger'])

    def getStepParameters(self):
        params = {'power': self.sbPower_2.value(),
                  'trigger': self.sbTime.value()}
        return params

    def setManualParameters(self, params):
        self.sbPower.setValue(params['power'])

    def getManualParameters(self):
        params = {'power': self.sbPower.value()}
        return params

    def setAutoParameters(self, params):
        self.sbWidth.setValue(params['width'])
        self.sbTrack.setValue(params['time'])
        self.auto_mode = params['mode']

    def getAutoParameters(self):
        params = {'width': self.sbWidth.value(),
                  'time': self.sbTrack.value(),
                  'mode': self.auto_mode}
        return params

    def btnModeClicked(self):
        if self.btnMode.currentText() == "Manual":
            self.lblStatus.setText("Manual")
            self.lblStatus.setStyleSheet(
                "background-color: rgb(255, 220, 0); color: rgb(0, 0, 0);")
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
                "background-color: rgb(128, 0, 128); color: rgb(255, 255, 255);")
            self.mode = STEP
            self.tbParams.setCurrentIndex(2)

    def btnAutoClicked(self):
        if self.btnAuto.currentText() == "Continuous":
            self.auto_mode = 0
            self.sbTrack.setEnabled(True)
        elif self.btnAuto.currentText() == "Tracks":
            self.sbTrack.setValue(0)
            self.auto_mode = 1
            self.sbTrack.setEnabled(False)


    def btnControlClicked(self):
        param = self.getParameters()
        rospy.set_param('/control/parameters', param)

        step = self.getStepParameters()
        rospy.set_param('/control/step', step)

        manual = self.getManualParameters()
        rospy.set_param('/control/manual', manual)

        auto = self.getAutoParameters()
        rospy.set_param('/control/automatic', auto)

        self.msg_control.change = True
        self.msg_control.value = self.mode
        self.pub_control.publish(self.msg_control)

    def btnCalibrateClicked(self):
        self.msg_calibrate.calibrate = 1
        self.pub_calibrate.publish(self.msg_calibrate)

    def cb_geometry(self, msg_geometry):
        self.minor_axis = msg_geometry.minor_axis
        self.major_axis = msg_geometry.major_axis

    def cb_power(self, msg_power):
        self.power = msg_power.value

    def cb_info(self, msg_info):
        self.track_number = msg_info.track_number
        self.time_control = msg_info.time

    def cb_start(self, msg_start):
        self.control = msg_start.control
        if self.control is True:
            self.sbWidth.setValue(msg_start.setpoint)
            auto = self.getAutoParameters()
            rospy.set_param('/control/automatic', auto)

    def tmrInfoEvent(self):
        self.lblInfo.setText(
            "Minor axis: %.2f<br>Major axis: %.2f<br><b>Power: %.0f</b>" % (
                self.minor_axis, self.major_axis, self.power))
        self.lblNumber.setText(str(self.track_number))
        self.lblTime_2.setText('%.2f' % float(self.time_control))
        if self.control is True:
            self.lblControl.setText("Activated")
            self.lblControl.setStyleSheet(
                 "background-color: rgb(0, 0, 128); color: rgb(255, 255, 255);")
        else:
            self.lblControl.setText("Stoped")
            self.lblControl.setStyleSheet(
                 "background-color: rgb(255, 0, 0); color: rgb(255, 255, 255);")


if __name__ == '__main__':
    rospy.init_node('control_panel')
    app = QtGui.QApplication(sys.argv)
    qt_control = QtControl()
    qt_control.show()
    app.exec_()
