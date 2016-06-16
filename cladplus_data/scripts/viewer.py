#!/usr/bin/env python
import sys
import os
import cv2
import numpy as np
import rospy
#import matplotlib.pyplot as plt

from PyQt4 import QtGui, QtCore, uic
#from defects import DefectsDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


DISPLAY_STYLE_SHEET = 'background-color: rgb(127,127,127); border: 3px solid rgb(63, 63, 63)'


class QDisplay(QtGui.QWidget):
    def __init__(self, parent=None, size=64):
        super(QDisplay, self).__init__(parent)
        self.parent = parent
        self.setMinimumSize(240, 240)

        layout = QtGui.QVBoxLayout()
        layout.setContentsMargins(1, 1, 1, 1)
        self.setLayout(layout)
        self.setMaximumSize(300, 300)

        self.lblCamera = QtGui.QLabel()
        self.lblCamera.setStyleSheet(DISPLAY_STYLE_SHEET)
        self.lblCamera.setSizePolicy(
            QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred,
                              QtGui.QSizePolicy.Expanding))
        self.lblCamera.setAlignment(QtCore.Qt.AlignCenter)
        self.lblCamera.mousePressEvent = self.mousePressEvent
        layout.addWidget(self.lblCamera)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.timeoutRunning)
        self.timer.start(20)
        self.first_data = False

        image_topic = rospy.get_param('~image', '/tachyon/image')
        rospy.Subscriber(image_topic, Image, self.cb_image, queue_size=1)
        self.bridge = CvBridge()

        self.lines_scale = 10
        self.width = size
        self.pixmap = QtGui.QPixmap()
        self.rect_size = 5, 5
        self.image = np.zeros((size, size), dtype=np.uint16)
        self.n = 0
        self.font = QtGui.QFont('Arial', self.width / 5)
        self.colo = QtGui.QColor(255, 255, 255)

    def updateFrame(self, image):
        self.image = image

    def paintFrame(self, image):
        if self.first_data:
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
            if not self.first_data:
                self.first_data = True
            frame = self.bridge.imgmsg_to_cv2(msg_image)
            self.updateFrame(frame)
        except CvBridgeError, e:
            print e


if __name__ == '__main__':
    rospy.init_node('visualize')
    app = QtGui.QApplication(sys.argv)
    img = cv2.imread('/home/panadeiro/catkin_ws/src/cladplus/cladplus_data/scripts/visualize/indice.jpeg')
    q = QDisplay()
    q.paintFrame(img)
    q.show()
    app.exec_()
