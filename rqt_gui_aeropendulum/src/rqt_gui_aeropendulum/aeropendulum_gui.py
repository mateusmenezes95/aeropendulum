import os
import datetime

import rospy
from rqt_gui_py.plugin import Plugin
from .aeropendulum_gui_widget import AeropendulumWidget

import csv

from aeropendulum_common_messages.srv import *
from aeropendulum_common_messages.msg import *

STEP_RESPONSE_MAX_TIME = 10

class Aeropendulum(Plugin):

    def __init__(self, context):
        super(Aeropendulum, self).__init__(context)
        self.setObjectName('NaoSei')
        # Give QObjects reasonable names
        self._widget = AeropendulumWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        rospy.Subscriber("graph_plot", GraphPlotData, self.plot)

        self.x = []
        self.yAngle = []
        self.ySetPointAngle = []
        self.count = 0
        self.period = 0.1
        self.plotStep = 50

        self.xSteadyState = []
        self.ySteadyState = []

        self._widget.setPointButton.clicked.connect(self.setPointRequest)
        self.setPointClient = rospy.ServiceProxy('set_point', SetPoint)

        self.stepResponseRunning = False
        self.plotGraph = False
        self._widget.stepResponseButton.clicked.connect(self.stepResponseRequest)
        self.stepResponseClient = rospy.ServiceProxy('unit_step_response', SetPoint)

        self.csvFilesCreated = False
        self._widget.csvButton.clicked.connect(self.createCsvFiles)

        self.steadyStateClient = rospy.ServiceProxy("steady_state", GetSteadyState)
        self._widget.connectionButton.clicked.connect(self.getSteadyStateFunc)

        # Create folder to store csv files
        csvFilesFolderName = 'aeropendulum_csv_files'
        desktopPath = os.path.join(os.path.expanduser('~'), 'Desktop')
        self.csvFolderPath = os.path.join(desktopPath, csvFilesFolderName)
        # Check if folder exists
        if not os.path.exists(self.csvFolderPath):
            os.makedirs(self.csvFolderPath)

    def plot(self, dataPlot):
        xTime = round((dataPlot.nSample * self.period), 3)
        dataPlot.angle = round(dataPlot.angle, 3)
        dataPlot.setPointAngle = round(dataPlot.setPointAngle, 3)
        self.x.append(xTime) 
        self.yAngle.append(dataPlot.angle)
        self.ySetPointAngle.append(dataPlot.setPointAngle)

        if self.stepResponseRunning:
            if xTime == STEP_RESPONSE_MAX_TIME:
                self.stepResponseRunning = False
                self.plotGraph = False
            csvData = [xTime, dataPlot.angle, dataPlot.setPointAngle]
            if self.csvFilesCreated:
                self.stepResponseCsvWriter.writerow(csvData)

        if self.plotGraph:
            # discards the old graph
            self._widget.ax.clear()

            if not self.stepResponseRunning:
                if self.count <= self.plotStep:
                    self._widget.ax.set_xlim([0, self.plotStep * self.period])
                else:
                    xMin = (self.count - (self.plotStep/2)) * self.period
                    xMax = (self.count + self.plotStep) * self.period
                    self._widget.ax.set_xlim([xMin, xMax])
            else:
                self._widget.ax.set_xlim([0, STEP_RESPONSE_MAX_TIME]) 
                
                
            
            yMin = self.ySetPointAngle[self.count] - 2
            yMax = self.ySetPointAngle[self.count] + 0.5
            self._widget.ax.set_ylim([yMin, yMax])

            # plot data
            self._widget.ax.plot(self.x, self.yAngle, 'r', label='Angulo atual')
            self._widget.ax.plot(self.x, self.ySetPointAngle, 'b', label='SetPoint')
            self._widget.ax.legend()

            # refresh canvas
            self._widget.canvas.draw()

        self.count += 1

    def setPointRequest(self):
        rospy.loginfo("Service client ok!")
        setPointValue = float(self._widget.setPointInput.text())
        rospy.loginfo("Sending setPoint %f", setPointValue)
        try:
            response = self.setPointClient(setPointValue, False)
            if response.done == True:
                rospy.loginfo("Response ok! SetPoint: %f", float(self._widget.setPointInput.text()))
            else:
                rospy.loginfo("Response wrong")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)

    def stepResponseRequest(self):
        rospy.loginfo("Step response requested")
        self.stepResponseRunning = True
        stepMagnitude = float(self._widget.setPointInput.text())
        try:
            response = self.stepResponseClient(stepMagnitude, True)
            if response.done == True:
                rospy.loginfo("Request sucess! SetPoint: %f", float(self._widget.setPointInput.text()))
            else:
                rospy.loginfo("Response wrong")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)

    def getSteadyStateFunc(self):
        try:
            response = self.steadyStateClient()
            xAxis = round(response.angle, 3)
            yAxis = round(response.controlSignal, 3)
            self.xSteadyState.append(xAxis)
            self.ySteadyState.append(yAxis)
            self._widget.ax.clear()
            self._widget.ax.plot(self.xSteadyState, self.ySteadyState)
            self._widget.canvas.draw()
            csvData = [xAxis, yAxis]
            if self.csvFilesCreated:
                self.steadyStateLineCsvWriter.writerow(csvData)
            rospy.loginfo("Steady State Ok! sinAngle: %f, controlSignal: %d", xAxis, yAxis)
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)

    def createCsvFiles(self):
        # Create csv file of current time for steady state line
        now = datetime.datetime.now()
        timeNow = str(now.day) + '_' + str(now.month) + '_' + str(now.hour) + '_' + str(now.minute) + '_' + str(now.second) 
        steadyStateLineCsvFileName = 'steady_state_line_' + timeNow
        steadyStateLineCsvFile = open(os.path.join(self.csvFolderPath, steadyStateLineCsvFileName), 'wb')
        self.steadyStateLineCsvWriter = csv.writer(steadyStateLineCsvFile, delimiter = ',')

        stepResponseCsvFileName = 'step_response_' + timeNow
        stepResponseCsvFile = open(os.path.join(self.csvFolderPath, stepResponseCsvFileName), 'wb')
        self.stepResponseCsvWriter = csv.writer(stepResponseCsvFile, delimiter = ',')

        aeropendulumOnCsvFileName = 'aeropendulum_on_' + timeNow
        aeropendulumOnCsvFile = open(os.path.join(self.csvFolderPath, aeropendulumOnCsvFileName), 'wb')
        self.aeropendulumOnCsvWriter = csv.writer(aeropendulumOnCsvFile, delimiter = ',')

        self.csvFilesCreated = True

    
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog