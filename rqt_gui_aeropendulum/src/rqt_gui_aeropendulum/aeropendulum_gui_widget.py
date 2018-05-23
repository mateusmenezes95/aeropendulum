import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon, QPixmap
from python_qt_binding.QtWidgets import QWidget

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import csv

import roslib
import rosmsg
import rospkg
import rospy

from aeropendulum_common_messages.msg import *
from aeropendulum_common_messages.srv import *


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

        self.setPointButton.clicked.connect(self.setPointRequest)
        self.setPointInput.setMaxLength(5)

        # Set icons images
        dirName = os.path.dirname(__file__)
        currentFolderRelativePath = '../../resource/icons/'
        
        powerIconName = 'power.svg'
        powerIconPath = os.path.normpath(os.path.join(dirName, currentFolderRelativePath, powerIconName))
        self.powerButton.setIcon(QIcon(powerIconPath))
        self.powerButton.setToolTip("Liga ou desliga o motor propulsor")

        stepResponseIconName = 'graphs.svg'
        stepResponseIconPath = os.path.normpath(os.path.join(dirName, currentFolderRelativePath, stepResponseIconName))
        self.stepResponseButton.setIcon(QIcon(stepResponseIconPath))
        self.stepResponseButton.setToolTip("Aplica um degrau unitario ao sistema")

        csvIconName = 'csv.svg'
        csvIconPath = os.path.normpath(os.path.join(dirName, currentFolderRelativePath, csvIconName))
        self.csvButton.setIcon(QIcon(csvIconPath))
        self.csvButton.setToolTip("Exporta dados para um arquivo CSV")

        connectionIconName = 'link.svg'
        connectionIconPath = os.path.normpath(os.path.join(dirName, currentFolderRelativePath, connectionIconName))
        self.connectionButton.setIcon(QIcon(connectionIconPath))
        self.connectionButton.setToolTip("Estabelecer conexao com o controlador")

        myFile = open('/home/menezes/Desktop/teste2.csv', 'wb')
        self.writer = csv.writer(myFile, delimiter = ',')

        self.setPointClient = rospy.ServiceProxy('set_setPoint', SetPoint)

        self.x = []
        self.yAngle = []
        self.ySetPointAngle = []
        self.count = 0
        self.period = 0.01
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

    def setPointRequest(self):
        rospy.loginfo("Service client ok!")
        setPointValue = float(self.setPointInput.text())
        rospy.loginfo("Sending setPoint %f", setPointValue)
        try:
            response = self.setPointClient(setPointValue)
            if response.done == True:
                rospy.loginfo("Response ok! SetPoint: %f", float(self.setPointInput.text()))
            else:
                rospy.loginfo("Response wrong")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)
