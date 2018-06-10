import os
import subprocess
import datetime
import threading

import rospy
import roslaunch

from rqt_gui_py.plugin import Plugin
from .aeropendulum_gui_widget import AeropendulumWidget
from python_qt_binding.QtCore import QTimer

import csv

from aeropendulum_common_messages.srv import *
from aeropendulum_common_messages.msg import *

STEP_RESPONSE_MAX_TIME = 20
DEFAULT_STEP_MAGNITUDE = 45
ARDUINO_PUBLISH_FREQUENCY = 100

class Aeropendulum(Plugin):

    def __init__(self, context):
        super(Aeropendulum, self).__init__(context)
        self.setObjectName('NaoSei')
        # Give QObjects reasonable names
        self._widget = AeropendulumWidget()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.graphPlotSub = rospy.Subscriber("graph_plot", GraphPlotData, self.getDataFromRealTime)
        # Timer to plot graph periodically
        timerPeriod = 5 * (1.0 / ARDUINO_PUBLISH_FREQUENCY)
        self.timer = QTimer()
        self.timer.timeout.connect(self.plot)
        self.timer.start(timerPeriod)

        # Parameters to graph plot
        self.yAxisMin = 0
        self.yAxisMax = 90.0

        self.count = 0
        self.period = 1.0 / ARDUINO_PUBLISH_FREQUENCY
        self.plotStep = 70


        # Create lists to store data in real time from aeropendulum
        self.x = []
        self.yAngle = []
        self.ySetPointAngle = []
        self.yAngleError = []
        self.yControlSignal = []

        # Create lists to store data from step response
        self.xStepResponse = []
        self.yStepResponseAngle = []
        self.yStepResponseSetPointAngle = []

        # Create lists to store data to steady state line
        self.xSteadyState = []
        self.ySteadyState = []

        self._widget.setPointButton.clicked.connect(self.setPointRequest)
        self.setPointClient = rospy.ServiceProxy('set_point', SetPoint)

        self.stepResponseRunning = False
        self.plotGraph = True
        self._widget.stepResponseButton.clicked.connect(self.stepResponseRequest)
        self.stepResponseClient = rospy.ServiceProxy('unit_step_response', SetPoint)

        self.csvFilesCreated = False
        self._widget.csvButton.clicked.connect(self.createCsvFiles)

        self.steadyStateClient = rospy.ServiceProxy("steady_state", GetSteadyState)
        # self._widget.connectionButton.clicked.connect(self.getSteadyStateFunc)

        # Create folder to store csv files
        csvFilesFolderName = 'aeropendulum_csv_files'
        desktopPath = os.path.join(os.path.expanduser('~'), 'Desktop')
        self.csvFolderPath = os.path.join(desktopPath, csvFilesFolderName)
        # Check if folder exists
        if not os.path.exists(self.csvFolderPath):
            os.makedirs(self.csvFolderPath)

    def plot(self):
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
            
            # plot data
            if not self.stepResponseRunning:
                self._widget.ax.plot(self.x, self.yAngle, 'r', label='Angulo atual')
                self._widget.ax.plot(self.x, self.ySetPointAngle, 'g', label='SetPoint')
                self._widget.ax.plot(self.x, self.yAngleError, 'b', label='Erro')
                self._widget.ax.plot(self.x, self.yControlSignal, 'y', label='Sinal de controle')
            else:
                self._widget.ax.plot(self.xStepResponse, self.yStepResponseAngle, 'r', label='Angulo atual')
                self._widget.ax.plot(self.xStepResponse, self.yStepResponseSetPointAngle, 'b', label='SetPoint')
            
            self._widget.ax.set_ylim([self.yAxisMin, self.yAxisMax])
            self._widget.ax.legend()

            # refresh canvas
            self._widget.canvas.draw()

    def setPointRequest(self):
        if self.stepResponseRunning:
            self.stepResponseRunning = False
        if not self.plotGraph:
            self.plotGraph = True
            self._widget.ax.clear()
        
        if self._widget.setPointInput.hasAcceptableInput():
            setPointValue = float(self._widget.setPointInput.text())
        else:
            setPointValue = DEFAULT_STEP_MAGNITUDE

        rospy.loginfo("Sending setPoint %f", setPointValue)

        try:
            response = self.setPointClient(setPointValue, False)
            if response.done == True:
                rospy.loginfo("Response ok! SetPoint: %f", setPointValue)
            else:
                rospy.loginfo("Response wrong")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)

    def getSteadyStateFunc(self):
        try:
            response = self.steadyStateClient()

            xAxis = round(response.angle, 3)
            yAxis = response.controlSignal

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
        now = datetime.datetime.now()
        timeNow = str(now.day) + '_' + str(now.month) + '_' + str(now.hour) + '_' + str(now.minute) + '_' + str(now.second) 

        steadyStateLineCsvFileName = 'steady_state_line_' + timeNow + '.csv'
        steadyStateLineCsvFilePath = os.path.join(self.csvFolderPath, steadyStateLineCsvFileName)
        self.steadyStateLineCsvFile = open(steadyStateLineCsvFilePath, 'wb')
        self.steadyStateLineCsvWriter = csv.writer(self.steadyStateLineCsvFile, delimiter = ',')

        stepResponseCsvFileName = 'step_response_' + timeNow + '.csv'
        stepResponseCsvFilePath = os.path.join(self.csvFolderPath, stepResponseCsvFileName)
        self.stepResponseCsvFile = open(stepResponseCsvFilePath, 'wb')
        self.stepResponseCsvWriter = csv.writer(self.stepResponseCsvFile, delimiter = ',')

        aeropendulumOnCsvFileName = 'aeropendulum_on_' + timeNow + '.csv'
        aeropendulumOnCsvFilePath = os.path.join(self.csvFolderPath, aeropendulumOnCsvFileName)
        self.aeropendulumOnCsvFile = open(aeropendulumOnCsvFilePath, 'wb')
        self.aeropendulumOnCsvWriter = csv.writer(self.aeropendulumOnCsvFile, delimiter = ',')

        self.csvFilesCreated = True
        rospy.loginfo("CSV Files created")
        rospy.loginfo("Create " + steadyStateLineCsvFileName + " in " + steadyStateLineCsvFilePath)
        rospy.loginfo("Create " + stepResponseCsvFileName + " in " + steadyStateLineCsvFilePath)
        rospy.loginfo("Create " + aeropendulumOnCsvFileName + " in " + aeropendulumOnCsvFilePath)

    def getDataFromRealTime(self, dataPlot):
        xTime = round((dataPlot.nSample * self.period), 3)
        dataPlot.angle = round(dataPlot.angle, 3)
        dataPlot.setPointAngle = round(dataPlot.setPointAngle, 3)
        dataPlot.angleError = round(dataPlot.angleError, 3)

        self._widget.actualAngleLabel.setText(str(dataPlot.angle))
        self._widget.setPointLabel.setText(str(dataPlot.setPointAngle))
        self._widget.errorLabel.setText(str(dataPlot.angleError))
        self._widget.controlSignalLabel.setText(str(dataPlot.controlSignal))

        self.x.append(xTime) 
        self.yAngle.append(dataPlot.angle)
        self.ySetPointAngle.append(dataPlot.setPointAngle)
        self.yAngleError.append(dataPlot.angleError)
        self.yControlSignal.append(dataPlot.controlSignal)

        if self.csvFilesCreated:
            CsvData = [xTime, dataPlot.angle, dataPlot.setPointAngle, dataPlot.angleError, dataPlot.controlSignal]
            self.aeropendulumOnCsvWriter.writerow(CsvData)

        self.count += 1

    def stepResponseRequest(self):
        self.xStepResponse = []
        self.yStepResponseAngle = []
        self.yStepResponseSetPointAngle = []
        
        # Clear lists and graph
        self._widget.ax.clear()
        self._widget.ax.set_xlim([0, STEP_RESPONSE_MAX_TIME])

        rospy.loginfo("Step response requested")
        if self._widget.setPointInput.hasAcceptableInput():
            stepMagnitude = float(self._widget.setPointInput.text())
        else:
            stepMagnitude = DEFAULT_STEP_MAGNITUDE
        try:
            response = self.stepResponseClient(stepMagnitude, True)
            if response.done == True:
                rospy.loginfo("Step response request sucess! Step Magnitude: %f", stepMagnitude)
            else:
                rospy.loginfo("Step response wrong")
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" %e)
        self.stepResponseRunning = True
        
        self.stepResponseSub = rospy.Subscriber("step_response", StepResponseData, self.getStepResponseData)        

    def getStepResponseData(self, data):
        xTime = round((data.nSample * self.period), 3)
        data.angle = round(data.angle, 3)
        data.setPointAngle = round(data.setPointAngle, 3)

        self.xStepResponse.append(xTime)
        self.yStepResponseAngle.append(data.angle)
        self.yStepResponseSetPointAngle.append(data.setPointAngle)

        if self.csvFilesCreated:
            stepResponseCsvData = [xTime, data.angle, data.setPointAngle]
            self.stepResponseCsvWriter.writerow(stepResponseCsvData)
        if xTime == STEP_RESPONSE_MAX_TIME:
            self.stepResponseSub.unregister()
            self.plotGraph = False
    
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog