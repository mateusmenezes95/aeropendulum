import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import random
import csv

import roslib
import rosmsg
import rospkg
import rospy
from aeropendulum_common_messages.msg import GraphPlotData


class AeropendulumWidget(QWidget):
    def __init__(self, parent=None):
        super(AeropendulumWidget, self).__init__(parent)

        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_gui_aeropendulum'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self)
        self.figure = Figure(facecolor = 'w')
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.graphLayout.addWidget(self.toolbar)
        self.graphLayout.addWidget(self.canvas)

        self.csvButton.setIcon(QIcon.fromTheme('document-new'))
        self.connectionButton.setIcon(QIcon.fromTheme('network-wired'))

        myFile = open('/home/menezes/Desktop/teste.csv', 'wb')
        self.writer = csv.writer(myFile, delimiter = ',')

        rospy.Subscriber("graph_plot", GraphPlotData, self.plot)
        # self.pub = rospy.Publisher('chatter', String, queue_size=10)

        self.x = []
        self.yAngle = []
        self.ySetPointAngle = []
        self.count = 0
        self.period = 0.1
        self.plotStep = 50

    def shutdown_plugin(self):
        print "teste!"
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def plot(self, dataPlot):
        # random data
        xData = dataPlot.nSample * self.period
        self.x.append(xData) 
        self.yAngle.append(dataPlot.angle)
        self.ySetPointAngle.append(dataPlot.setPointAngle)

        csvData = [xData, dataPlot.angle]
        self.writer.writerow(csvData)

        # create an axis

        # discards the old graph
        self.ax.clear()

        if self.count <= self.plotStep:
            self.ax.set_xlim([0, self.plotStep * self.period])
        else:
            xMin = (self.count - (self.plotStep/2)) * self.period
            xMax = (self.count + self.plotStep) * self.period
            self.ax.set_xlim([xMin, xMax])
        
        yMin = self.ySetPointAngle[self.count] - 2
        yMax = self.ySetPointAngle[self.count] + 0.5
        self.ax.set_ylim([yMin, yMax])

        # plot data
        self.ax.plot(self.x, self.yAngle, 'r', label='Angulo atual')
        self.ax.plot(self.x, self.ySetPointAngle, 'b', label='SetPoint')
        self.ax.legend()

        # refresh canvas
        self.canvas.draw()

        self.count += 1