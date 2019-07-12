import sys

from plotWindow import PlotWindow
from controlsWindow import ControlsWindow

from PyQt5.QtWidgets import *


"""
Program start point. This class handles all child windows of the gui
"""
class GUI:

    def __init__(self):

        self.screenResolution = [app.desktop().screenGeometry().width(),app.desktop().screenGeometry().height()]

        self.controlsWindow = ControlsWindow(self)
        self.plotWindow = PlotWindow()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
