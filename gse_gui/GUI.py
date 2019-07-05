import sys

from plotWindow import PlotWindow
from controlsWindow import ControlsWindow

from PyQt5.QtWidgets import *


class GUI():
    """
        Parent class to hold all child windows
    """

    def __init__(self, parent=None):

        self.controlsWindow = ControlsWindow(self)
        self.plotWindow = PlotWindow()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
