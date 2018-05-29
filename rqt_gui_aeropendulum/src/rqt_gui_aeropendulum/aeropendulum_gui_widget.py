import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QIcon, QPixmap, QDoubleValidator
from python_qt_binding.QtWidgets import QWidget

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

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
        # self.toolbar = NavigationToolbar(self.canvas, self)
        # self.graphLayout.addWidget(self.toolbar)
        self.graphLayout.addWidget(self.canvas)

        self.setPointInput.setMaxLength(5)
        self.setPointInput.setValidator(QDoubleValidator(0.00, 90.00, 2))

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
        self.csvButton.setToolTip("Cria arquivos CSV")

        connectionIconName = 'link.svg'
        connectionIconPath = os.path.normpath(os.path.join(dirName, currentFolderRelativePath, connectionIconName))
        self.connectionButton.setIcon(QIcon(connectionIconPath))
        self.connectionButton.setToolTip("Estabelecer conexao com o controlador")

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
