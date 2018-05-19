import rospy
from rqt_gui_py.plugin import Plugin
from .aeropendulum_gui_widget import AeropendulumWidget

# from .data_plot import DataPlot

class Aeropendulum(Plugin):

    def __init__(self, context):
        super(Aeropendulum, self).__init__(context)
        self.setObjectName('NaoSei')
        # Give QObjects reasonable names
        self._widget = AeropendulumWidget()
        self._widget.kParametersButton.clicked.connect(self._widget.plot)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

    def testeFunc(self):
        print "teste"
        self._widget.pub.publish("Saporra!")

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog