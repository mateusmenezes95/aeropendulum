import rospy
from rqt_gui_py.plugin import Plugin
from .aeropendulum_gui_widget import AeropendulumWidget

from aeropendulum_common_messages.srv import *
from aeropendulum_common_messages.msg import *

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
        self.period = 0.01
        self.plotStep = 50

        self._widget.setPointButton.clicked.connect(self.setPointRequest)
        self.setPointClient = rospy.ServiceProxy('set_setPoint', SetPoint)

        writeCsvFile = False
        self._widget.stepResponseButton.clicked.connect(self.stepResponseRequest)
        self.stepResponseClient = rospy.ServiceProxy('unit_step_response', SetPoint)

    def plot(self, dataPlot):
        xData = dataPlot.nSample * self.period
        self.x.append(xData) 
        self.yAngle.append(dataPlot.angle)
        self.ySetPointAngle.append(dataPlot.setPointAngle)

        csvData = [xData, dataPlot.angle]
        self.writer.writerow(csvData)

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
        setPointValue = float(self._widget.setPointInput.text())
        rospy.loginfo("Sending setPoint %f", setPointValue)
        try:
            response = self.setPointClient(setPointValue)
            if response.done == True:
                rospy.loginfo("Response ok! SetPoint: %f", float(self.setPointInput.text()))
            else:
                rospy.loginfo("Response wrong")
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s" %e)

    def stepResponseRequest(self):
        pass
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog