#!/usr/bin/env python
import sys
import cv2
import rospy
import numpy as np

from python_qt_binding import QtGui
from python_qt_binding import QtCore

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class QtDisplay(QtGui.QWidget):
    def __init__(self, parent=None, size=64):
        super(QtDisplay, self).__init__(parent)
        self.parent = parent
        self.setMinimumSize(240, 240)

        layout = QtGui.QVBoxLayout()
        layout.setContentsMargins(1, 1, 1, 1)
        self.setLayout(layout)
        self.setMaximumSize(300, 300)

        self.lblCamera = QtGui.QLabel()
        self.lblCamera.setStyleSheet(
            'background-color: rgb(127,127,127); border: 3px solid rgb(63, 63, 63)')
        self.lblCamera.setSizePolicy(
            QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred,
                              QtGui.QSizePolicy.Expanding))
        self.lblCamera.setAlignment(QtCore.Qt.AlignCenter)
        self.lblCamera.mousePressEvent = self.mousePressEvent
        layout.addWidget(self.lblCamera)

        # self.timer = QtCore.QTimer(self)
        # self.timer.timeout.connect(self.timeoutRunning)
        # self.timer.start(20)

        image_topic = rospy.get_param('~image', '/tachyon/image')
        rospy.Subscriber(image_topic, Image, self.cb_image, queue_size=1)
        self.bridge = CvBridge()

        size = 32
        self.pixmap = QtGui.QPixmap()
        self.image = np.zeros((size, size, 3), dtype=np.uint16)
        self.font = QtGui.QFont('Arial', size / 5)
        self.colo = QtGui.QColor(255, 255, 255)

    def paintFrame(self, image):
        height, width, channels = image.shape
        image = cv2.resize(image, (width*2, height*2))
        height, width, channels = image.shape
        image = QtGui.QImage(image.tostring(), width, height,
                             channels * width, QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter()
        pixmap = self.pixmap.scaled(self.lblCamera.size(),
                                    QtCore.Qt.KeepAspectRatio)
        painter = QtGui.QPainter()
        self.pixmap.convertFromImage(image)
        painter.begin(self.pixmap)
        painter.setWindow(0, 0, 32, 32)
        painter.end()
        self.lblCamera.setPixmap(pixmap)

    def timeoutRunning(self):
        self.paintFrame(self.image)

    def cb_image(self, msg_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg_image)
            self.image = frame
        except CvBridgeError, e:
            print e


if __name__ == '__main__':
    rospy.init_node('display')

    # img = cv2.imread('/home/panadeiro/catkin_ws/src/cladplus/cladplus_data/scripts/visualize/indice.jpeg')
    app = QtGui.QApplication(sys.argv)
    qt_display = QtDisplay()

    timer = QtCore.QTimer()
    timer.timeout.connect(qt_display.timeoutRunning)
    timer.start(20)

    qt_display.show()
    app.exec_()
