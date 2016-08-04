#!/usr/bin/python
# -*- coding: utf-8 -*-
# plotting control results
# Autor: Veronica Panadeiro Castro

import numpy as np
import rosbag

import matplotlib.pyplot as plt


class QtPlot():
    def __init__(self, filename=None):
        self.min_meas, self.max_meas = 0, 5
        self.min_power, self.max_power = 0, 1500
        self.reset_data()
        self.getdata(filename)
        self.plot()

    def reset_data(self):
        self.geometry = np.array([])
        self.control = np.array([])

    def getdata(self, filename):
        self.reset_data()
        bag = rosbag.Bag(filename)
        data = []
        geometry = []
        control = []
        for idx, (topic, msg, mt) in enumerate(bag.read_messages(topics='/tachyon/geometry')):
            data1 = msg.header.stamp.to_sec()
            data2 = msg.minor_axis
            data.append(data1)
            data.append(data2)
            geometry.append(data)
            data = []
        for idx, (topic, msg, mt) in enumerate(bag.read_messages(topics='/control/power'),):
            data1 = msg.header.stamp.to_sec()
            data2 = msg.value
            data.append(data1)
            data.append(data2)
            control.append(data)
            data = []
        bag.close()

        self.geometry = np.array(geometry)
        self.control = np.array(control)
        index, index_f, step, fstep = self.detect_step(self.geometry)
        self.geometry = self.geometry[(index[0]-10):(index_f[0]+10)]
        #Buscando o punto co mesmo time stamp
        c_s = np.where(self.control == step[0][0])[0]
        c_f = np.where(self.control == fstep[0][0])[0]
        #Collese a parte de control que nos interesa
        self.control = self.control[(c_s[0]-10):(c_f[0]+10)]
        # Comezase a contar no tempo cero
        self.geometry[:, 0] = self.geometry[:, 0]-self.geometry[0, 0]
        self.control[:, 0] = self.control[:, 0]-self.control[0, 0]

    def plot(self):
        plt.figure(figsize=(6, 4))
        width = plt.subplot(2, 1, 1)
        width.set_ylim(self.min_meas, self.max_meas)
        width.plot(self.geometry[:, 0], self.geometry[:, 1], color="blue",
                   linewidth=1.0, linestyle="-")
        power = plt.subplot(2, 1, 2)
        power.set_ylim(self.min_power, self.max_power)
        power.plot(self.control[:, 0], self.control[:, 1], color="red",
                   linewidth=1.0, linestyle="-")
        plt.show()

    def detect_step(self, data):
        p_data = [0, 0]
        step = []
        fstep = []
        for frame in data:
            if frame[1]-p_data[1] > 1.0:
                step.append(frame)
            if p_data[1]-frame[1] > 1.0:
                fstep.append(p_data)
            p_data = frame
        index = np.where(data == step[0])[0]
        index_f = np.where(data == fstep[0])[0]
        return index, index_f, step, fstep


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=str,
                        default=None,
                        help='path to input file')
    args = parser.parse_args()
    filename = args.file
    qt_plot = QtPlot(filename)
