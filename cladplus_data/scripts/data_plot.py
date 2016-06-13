#!/usr/bin/env python
import os
import sys
import time
import datetime
import numpy as np
import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib  import gridspec
from matplotlib.lines import Line2D
from PyQt4 import QtCore, QtGui, uic
import rospy
from collections import deque
from mashes_measures.msg import MsgGeometry
from mashes_control.msg import MsgPower


class Filter():
    def __init__(self, fc=100):
        self.fc = fc
        self.y = 0
        self.t = 0

    def update(self, x, t):
        DT = t - self.t
        a = (2 * np.pi * DT * self.fc) / (2 * np.pi * DT * self.fc + 1)
        y = a * x + (1 - a) * self.y
        self.y = y
        self.t = t
        return y


class AlasPlot(QtGui.QWidget):
    def __init__(self, parent=None):
        super(AlasPlot, self).__init__(parent)
        self.time_first = 0
        self.time_first_p = 0

        self.fig = Figure(figsize=(9, 6), dpi=72, facecolor=(0.76, 0.78, 0.8), edgecolor=(0.1, 0.1, 0.1), linewidth=2)
        self.canvas = FigureCanvas(self.fig)
        gs = gridspec.GridSpec(5, 1)
        self.plot1_axis1 = self.fig.add_subplot(gs[0:2, 0])
        self.plot1_axis1.get_yaxis().set_ticklabels([])
        self.plot2_axis1 = self.fig.add_subplot(gs[3:5, 0])

        self.tmrMeasures = QtCore.QTimer(self)
        self.tmrMeasures.timeout.connect(self.timeMeasuresEvent)
        self.tmrMeasures.start(100)


        rospy.Subscriber('/tachyon/geometry', MsgGeometry, self.measures)
        rospy.Subscriber('/control/power', MsgPower, self.power_fn)

        layout = QtGui.QHBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.canvas)

        self.PIX_SCALE = 31
        self.min_meas, self.max_meas = 0 / self.PIX_SCALE, 500 / self.PIX_SCALE
        self.min_power, self.max_power = 0, 2000

        self.distance = 0
        self.distance_p = 0
        self.duration = 10
        self.reset_data()

        self.line_mwidth = Line2D(self.wid_x, self.mwidth, color='b', linewidth=2, animated=True)
        self.text_mwidth = self.plot1_axis1.text(self.duration-10, 0, '',  size=13, ha='right', va='center', backgroundcolor='w', color='b', animated=True)

        self.plot2_axis1.set_ylim(self.min_power, self.max_power)
        self.line_power = Line2D(self.power_x, self.power, color='r', linewidth=2, animated=True)
        self.text_power = self.plot2_axis1.text(self.duration-10, 0, '',  size=13, ha='right', va='center', backgroundcolor='w', color='r', animated=True)
        self.draw_figure()


    def reset_data(self):
        self.str_data = ''
        self.mwidth = np.array([])
        self.power = np.array([])
        self.power_x = np.array([])
        self.wid_x = np.array([])

        self.mwidth_filter = Filter()
        self.power_filter = Filter()

    def draw_figure(self):
        self.plot1_axis1.cla()
        self.plot1_axis1.set_title('Melt Pool Measures')

        self.plot1_axis1.add_line(self.line_mwidth)
        self.plot1_axis1.set_xlim(0, self.duration)
        self.plot1_axis1.set_ylabel('Measures (mm)')
        self.plot1_axis1.set_xlabel('Time (s)')
        self.plot1_axis1.set_ylim(self.min_meas, self.max_meas)
        self.plot1_axis1.grid(True)

        self.plot2_axis1.set_title('Power')
        # self.plot2_axis1.get_xaxis().set_ticklabels([])
        self.plot2_axis1.set_xlabel('Time (s)')
        self.plot2_axis1.set_xlim(0, self.duration)
        self.plot2_axis1.add_line(self.line_power)
        self.plot2_axis1.set_ylabel('Power (W)')
        self.plot2_axis1.set_ylim(self.min_power, self.max_power)
        self.plot2_axis1.grid(True)

        self.canvas.draw()

        self.figbackground = self.canvas.copy_from_bbox(self.fig.bbox)
        self.background1 = self.canvas.copy_from_bbox(self.plot1_axis1.bbox)
        self.background2 = self.canvas.copy_from_bbox(self.plot2_axis1.bbox)

    def resizeEvent(self, event):
        self.figbackground = None
        self.background1 = None
        self.background2 = None

    def _limited_range(self, value, min_value, max_value):
        if value < min_value:
            value = min_value
        elif value > max_value:
            value = max_value
        return value

    def measures(self, msg_geometry):
        if self.time_first == 0:
            self.time_first = msg_geometry.header.stamp.to_sec()

        if msg_geometry.header.stamp.to_sec()-self.time_first > self.duration:
            self.distance = msg_geometry.header.stamp.to_sec()-self.time_first-self.duration

        self.wid_x = np.append(self.wid_x, msg_geometry.header.stamp.to_sec()-self.time_first)
        self.mwidth = np.append(self.mwidth, msg_geometry.minor_axis)

        self.line_mwidth.set_data(self.wid_x-self.distance, self.mwidth)
        if len(self.wid_x) > 2:
            steps = self.wid_x[-1]-self.wid_x[-2]
            # Melt pool measures filtered value
            mwidth_mean = np.round(self.mwidth_filter.update(self.mwidth[-1], steps), 1)
            self.text_mwidth.set_text('%.1f' % mwidth_mean)
            mwidth_mean = self._limited_range(mwidth_mean, self.min_meas + 20. / self.PIX_SCALE, self.max_meas -  20. / self.PIX_SCALE)
            self.text_mwidth.set_y(mwidth_mean)
            self.text_mwidth.set_x(self.duration-0.1*(self.duration))


    def power_fn(self, msg_power):
        if self.time_first_p == 0:
            self.time_first_p = msg_power.header.stamp.to_sec()

        if msg_power.header.stamp.to_sec()-self.time_first_p > self.duration:
            self.distance_p = msg_power.header.stamp.to_sec()-self.time_first_p-self.duration

        self.power_x = np.append(self.power_x, msg_power.header.stamp.to_sec()-self.time_first_p)
        self.power = np.append(self.power, msg_power.value)
        self.line_power.set_data(self.power_x - self.distance_p, self.power)
        if len(self.power_x) > 2:
            steps = self.power_x[-1]-self.power_x[-2]
            # print 'steps power:', steps
            power_mean = np.round(self.power_filter.update(self.power[-1], steps), 0)
            self.text_power.set_text('%.0f W' % power_mean)
            power_mean = self._limited_range(power_mean, self.min_power + 30. / self.PIX_SCALE, self.max_power - 30. / self.PIX_SCALE)
            self.text_power.set_y(power_mean)
            self.text_power.set_x(self.duration-0.1*(self.duration))

    def time_event(self):
        if self.figbackground == None or self.background1 == None or self.background2 == None:
            self.draw_figure()
        self.canvas.restore_region(self.figbackground)

        self.canvas.restore_region(self.background1)
        self.plot1_axis1.draw_artist(self.line_mwidth)
        self.plot1_axis1.draw_artist(self.text_mwidth)
        self.canvas.blit(self.plot1_axis1.bbox)
        self.canvas.restore_region(self.background2)

        self.plot2_axis1.draw_artist(self.line_power)
        self.plot2_axis1.draw_artist(self.text_power)
        self.canvas.blit(self.plot2_axis1.bbox)

    def timeMeasuresEvent(self):
        self.time_event()


if __name__ == "__main__":
    rospy.init_node('data_plot')
    app = QtGui.QApplication(sys.argv)
    alasPlot = AlasPlot()
    alasPlot.show()
    app.exec_()
