#!/usr/bin/env python
import sys
import cv2
import rospy
import numpy as np

from python_qt_binding import QtGui
from python_qt_binding import QtCore

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def thermal_colormap(levels=1024):
    colors = np.array([[0.00, 0.00, 0.00],   # black
                       [0.19, 0.00, 0.55],   # purple
                       [0.55, 0.00, 0.62],   # violet
                       [0.78, 0.05, 0.55],   # violet-pink
                       [0.90, 0.27, 0.10],   # red-orange
                       [0.96, 0.47, 0.00],   # orange
                       [1.00, 0.70, 0.00],   # yellow-orange
                       [1.00, 0.90, 0.20],   # yellow
                       [1.00, 1.00, 1.00]])  # white
    steps = levels / (len(colors)-1)
    lut = []
    for c in range(3):
        col = []
        for k in range(1, len(colors)):
            col.append(np.linspace(colors[k-1][c], colors[k][c], steps))
        col = np.concatenate(col)
        lut.append(col)
    lut = np.transpose(np.vstack(lut))
    lut_iron = np.uint8(lut * 255)
    return lut_iron

LUT_IRON = thermal_colormap()


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

        image_topic = rospy.get_param('~image', '/tachyon/image')
        rospy.Subscriber(image_topic, Image, self.cb_image, queue_size=1)
        self.bridge = CvBridge()

        size = 32
        self.pixmap = QtGui.QPixmap()
        self.image = np.zeros((size, size, 3), dtype=np.uint8)

    def paintFrame(self, image):
        if len(image.shape) == 2:
            image = LUT_IRON[image]
        height, width, channels = image.shape
        width, height = 2 * width, 2 * height
        image = cv2.resize(image, (width, height))
        image = QtGui.QImage(image.tostring(), width, height,
                             channels * width, QtGui.QImage.Format_RGB888)
        self.pixmap.convertFromImage(image)
        pixmap = self.pixmap.scaled(self.lblCamera.size(),
                                    QtCore.Qt.KeepAspectRatio)
        painter = QtGui.QPainter()
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

    # img = cv2.imread('indice.jpeg')
    app = QtGui.QApplication(sys.argv)
    qt_display = QtDisplay()

    timer = QtCore.QTimer()
    timer.timeout.connect(qt_display.timeoutRunning)
    timer.start(20)

    qt_display.show()
    app.exec_()
