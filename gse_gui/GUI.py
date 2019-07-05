import sys

from PlotWindow import PlotWindow
from ControlsWindow import ControlsWindow

from PyQt5.QtWidgets import *


class GUI():
    """
        Parent class to hold all child windows
    """
    screenResolution = []

    def __init__(self, parent=None):

        self.screenResolution = [app.desktop().screenGeometry().width(),app.desktop().screenGeometry().height()]

        print(self.screenResolution)

        self.controlsWindow = ControlsWindow(self)
        self.plotWindow = PlotWindow()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
