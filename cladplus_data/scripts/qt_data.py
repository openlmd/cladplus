#!/usr/bin/env python
import os
import sys
import time
import yaml
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from mashes_measures.msg import MsgStatus


HOME = os.path.expanduser('~')
DIRDATA = 'bag_data'
DIRDEST = 'data'

TOPICS = ['/tachyon/image',
          '/tachyon/geometry',
        #   '/control/power',
          '/predict/power']


class QtData(QtGui.QWidget):
    accepted = QtCore.pyqtSignal(list)

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        path = rospkg.RosPack().get_path('cladplus_data')
        loadUi(os.path.join(path, 'resources', 'data.ui'), self)

        self.btnJob.clicked.connect(self.btnJobClicked)
        self.btnRecord.clicked.connect(self.btnRecordClicked)

        dirdata = os.path.join(HOME, DIRDATA)
        if not os.path.exists(dirdata):
            os.mkdir(dirdata)

        self.job = ''
        self.name = ''
        self.status = True
        self.process = QtCore.QProcess(self)
        self.btnJobClicked()

    def btnJobClicked(self):
        self.job = self.txtJobName.text()
        dirname = os.path.join(HOME, DIRDATA, self.job)
        self.dirdata = dirname
        if not os.path.exists(self.dirdata):
             os.mkdir(self.dirdata)

    def btnRecordClicked(self):
        self.record()
        if self.status:
            self.btnJobClicked()
            self.btnRecord.setText('Recording...')
            if not os.path.exists(self.dirdata):
                os.mkdir(self.dirdata)
            self.txtOutput.textCursor().insertText('> ready for data.\n')
            self.accepted.emit([])
        else:
            self.btnRecord.setText('Record Data')
            self.txtOutput.textCursor().insertText(
            '> stopped.\n')
        self.status = not self.status

    def callProgram(self):
        os.chdir(self.dirdata)
        filename = self.name + '.bag'
        self.recording = True
        self.process.start(
            'rosrun rosbag record -O %s %s' % (filename, ' '.join(TOPICS)))

    def killProgram(self):
        os.system('killall -2 record')
        self.process.waitForFinished()
        self.recording = False

    def record(self):
        if self.status:
            self.name = time.strftime('%Y%m%d-%H%M%S')
            self.txtOutput.textCursor().insertText(
                '> recording topics:\n%s\n' % '\n'.join(TOPICS))
            self.callProgram()
        else:
            self.killProgram()
            self.txtOutput.textCursor().insertText(
                '> %s recorded.\n' % self.name)
        self.txtOutput.moveCursor(QtGui.QTextCursor.End)
        self.txtOutput.ensureCursorVisible()


if __name__ == "__main__":
    rospy.init_node('data_panel')

    app = QtGui.QApplication(sys.argv)
    qt_data = QtData()
    qt_data.show()
    app.exec_()
