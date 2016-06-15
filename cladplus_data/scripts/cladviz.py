#!/usr/bin/env python
import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

import rviz

import numpy as np
from mashes_measures.msg import MsgStatus

from qt_plot import QtPlot
from qt_control import QtControl
from viewer import QDisplay

path = rospkg.RosPack().get_path('cladplus_data')


class CladViz(QtGui.QMainWindow):
    def __init__(self):
        super(CladViz, self).__init__()
        loadUi(os.path.join(path, 'resources', 'cladviz.ui'), self)

        self.boxPlot.addWidget(QtPlot())
        self.vl_display.addWidget(QDisplay())

        self.qtControl = QtControl()
        self.tabWidget.addTab(self.qtControl, 'Control')

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        rospy.Subscriber('/supervisor/status', MsgStatus, self.cb_status, queue_size=1)

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
