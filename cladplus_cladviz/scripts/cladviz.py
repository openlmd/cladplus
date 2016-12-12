#!/usr/bin/env python
import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from mashes_measures.msg import MsgStatus

from qt_plot import QtPlot
from qt_control import QtControl
from qt_display import QtDisplay


class CladViz(QtGui.QMainWindow):
    def __init__(self):
        super(CladViz, self).__init__()
        path = rospkg.RosPack().get_path('cladplus_cladviz')
        loadUi(os.path.join(path, 'resources', 'cladviz.ui'), self)

        self.qt_plot = QtPlot()
        self.qt_display = QtDisplay()
        self.boxPlot.addWidget(self.qt_plot)
        self.vl_display.addWidget(self.qt_display)

        self.qtControl = QtControl()
        self.tabWidget.addTab(self.qtControl, 'Control')

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timeoutRunning)
        self.timer.start(100)

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        rospy.Subscriber(
            '/supervisor/status', MsgStatus, self.cb_status, queue_size=1)

    def timeoutRunning(self):
        self.qt_display.timeoutRunning()
        self.qt_plot.timeMeasuresEvent()

    def cb_status(self, msg_status):
        txt_status = ''
        if msg_status.laser_on:
            txt_status = 'Laser ON' + '\n'
            # self.lblStatus.setStyleSheet(
            #     "background-color: rgb(255, 255, 0); color: rgb(0, 0, 0);")
        else:
            txt_status = 'Laser OFF' + '\n'
            # self.lblStatus.setStyleSheet(
            #     "background-color: rgb(255, 255, 0); color: rgb(0, 0, 0);")
        self.lblStatus.setText(txt_status)

    def btnQuitClicked(self):
        QtCore.QCoreApplication.instance().quit()


if __name__ == '__main__':
    import sys

    rospy.init_node('cladviz')

    app = QtGui.QApplication(sys.argv)
    cladviz = CladViz()
    cladviz.show()
    sys.exit(app.exec_())
