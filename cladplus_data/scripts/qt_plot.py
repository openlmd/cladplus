#!/usr/bin/env python
import sys
import rospy
import numpy as np

from python_qt_binding import QtGui
from python_qt_binding import QtCore

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib import gridspec
from matplotlib.lines import Line2D

from mashes_measures.msg import MsgGeometry
from cladplus_control.msg import MsgPower


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


class QtPlot(QtGui.QWidget):
    def __init__(self, parent=None):
        super(QtPlot, self).__init__(parent)
        self.fig = Figure(figsize=(9, 6), dpi=72, facecolor=(0.76, 0.78, 0.8),
                          edgecolor=(0.1, 0.1, 0.1), linewidth=2)
        self.canvas = FigureCanvas(self.fig)
        gs = gridspec.GridSpec(5, 1)
        self.plot1_axis1 = self.fig.add_subplot(gs[0:2, 0])
        self.plot1_axis1.get_yaxis().set_ticklabels([])
        self.plot2_axis1 = self.fig.add_subplot(gs[3:5, 0])

        layout = QtGui.QHBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.canvas)


        rospy.Subscriber('/tachyon/geometry', MsgGeometry, self.measures)
        rospy.Subscriber('/control/power', MsgPower, self.power_fn)

        self.min_meas, self.max_meas = 0, 5
        self.min_power, self.max_power = 0, 2000

        self.time = 0
        self.distance = 0
        self.distance_p = 0
        self.duration = 6
        self.buff_max = self.duration * 500
        self.reset_data()

        self.line_width = Line2D(
            self.wtime, self.width, color='b', linewidth=2, animated=True)
        self.text_width = self.plot1_axis1.text(
            self.duration-10, 0, '', size=13, ha='right', va='center',
            backgroundcolor='w', color='b', animated=True)

        self.plot2_axis1.set_ylim(self.min_power, self.max_power)
        self.line_power = Line2D(
            self.ptime, self.power, color='r', linewidth=2, animated=True)
        self.text_power = self.plot2_axis1.text(
            self.duration-10, 0, '', size=13, ha='right', va='center',
            backgroundcolor='w', color='r', animated=True)
        self.draw_figure()

    def reset_data(self):
        self.width = []
        self.wtime = []
        self.width_filter = Filter()
        self.power = []
        self.ptime = []
        self.power_filter = Filter()

    def draw_figure(self):
        self.plot1_axis1.cla()
        self.plot1_axis1.set_title('Melt Pool Measures')

        self.plot1_axis1.add_line(self.line_width)
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
        time = msg_geometry.header.stamp.to_sec()
        if self.time == 0 or self.time > time:
            self.time = time
            self.reset_data()
        if time-self.time > self.duration:
            self.distance = time-self.time-self.duration
        self.wtime.append(time-self.time)
        self.width.append(msg_geometry.minor_axis)
        self.line_width.set_data(np.array(self.wtime)-self.distance, self.width)
        if len(self.width) > self.buff_max:
            self.mea_buffers()
        if len(self.wtime) > 2:
            # Melt pool measures filtered value
            width_mean = np.round(self.width_filter.update(
                self.width[-1], self.wtime[-1]), 1)
            self.text_width.set_text('%.1f' % width_mean)
            width_mean = self._limited_range(width_mean, 0.5, 4.5)
            self.text_width.set_y(width_mean)
            self.text_width.set_x(0.98 * self.duration)

    def power_fn(self, msg_power):
        time = msg_power.header.stamp.to_sec()
        if self.time == 0 or self.time > time:
            self.time = time
            self.reset_data()
        if time-self.time > self.duration:
            self.distance_p = time-self.time-self.duration
        self.ptime.append(time-self.time)
        self.power.append(msg_power.value)
        self.line_power.set_data(np.array(self.ptime)-self.distance_p, self.power)
        if len(self.power) > self.buff_max:
            self.pow_buffers()
        if len(self.ptime) > 2:
            power_mean = np.round(
                self.power_filter.update(self.power[-1], self.ptime[-1]), 0)
            self.text_power.set_text('%.0f W' % power_mean)
            power_mean = self._limited_range(power_mean, 100, 1900)
            self.text_power.set_y(power_mean)
            self.text_power.set_x(0.98 * self.duration)

    def timeMeasuresEvent(self):
        if self.figbackground == None or self.background1 == None or self.background2 == None:
            self.draw_figure()
        self.canvas.restore_region(self.figbackground)
        self.canvas.restore_region(self.background1)
        self.plot1_axis1.draw_artist(self.line_width)
        self.plot1_axis1.draw_artist(self.text_width)
        self.canvas.blit(self.plot1_axis1.bbox)
        self.canvas.restore_region(self.background2)
        self.plot2_axis1.draw_artist(self.line_power)
        self.plot2_axis1.draw_artist(self.text_power)
        self.canvas.blit(self.plot2_axis1.bbox)

    def mea_buffers(self):
        self.wtime = self.wtime[-(self.buff_max-1):]
        self.width = self.width[-(self.buff_max-1):]


    def pow_buffers(self):
        self.ptime = self.ptime[-(self.buff_max-1):]
        self.power = self.power[-(self.buff_max-1):]


if __name__ == "__main__":
    rospy.init_node('data_plot')
    app = QtGui.QApplication(sys.argv)
    qt_plot = QtPlot()
    tmrMeasures = QtCore.QTimer()
    tmrMeasures.timeout.connect(qt_plot.timeMeasuresEvent)
    tmrMeasures.start(100)
    qt_plot.show()
    app.exec_()
