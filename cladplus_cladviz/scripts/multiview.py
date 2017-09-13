#!/usr/bin/env python
import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from mashes_measures.msg import MsgStatus

from qt_multidisplay import QtMultiDisplay


class MultiView(QtGui.QMainWindow):
    def __init__(self):
        super(MultiView, self).__init__()
        path = rospkg.RosPack().get_path('cladplus_cladviz')
        loadUi(os.path.join(path, 'resources', 'multiview.ui'), self)

        self.qt_multidisplay = QtMultiDisplay(topic='/core_0/image')
        self.vl_display0.addWidget(self.qt_multidisplay)
        self.qt_multidisplay_1 = QtMultiDisplay(topic='/core_1/image')
        self.vl_display1.addWidget(self.qt_multidisplay_1)
        self.qt_multidisplay_2 = QtMultiDisplay(topic='/core_2/image')
        self.vl_display2.addWidget(self.qt_multidisplay_2)
        self.qt_multidisplay_3 = QtMultiDisplay(topic='/core_3/image')
        self.vl_display3.addWidget(self.qt_multidisplay_3)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timeoutRunning)
        self.timer.start(100)

        self.btnQuit.clicked.connect(self.btnQuitClicked)

        rospy.Subscriber(
            '/supervisor/status', MsgStatus, self.cb_status, queue_size=1)

    def timeoutRunning(self):
        self.qt_multidisplay.timeoutRunning()
        self.qt_multidisplay_1.timeoutRunning()
        self.qt_multidisplay_2.timeoutRunning()
        self.qt_multidisplay_3.timeoutRunning()

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
    cladviz = MultiView()
    cladviz.show()
    sys.exit(app.exec_())
