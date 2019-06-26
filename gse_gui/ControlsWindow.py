from PyQt5.QtWidgets import *

from SolenoidWidget import SolenoidWidget

class ControlsWindow(QMainWindow):
    """
        Window for control of system
        """

    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        # Set geometry
        self.title = 'Controls Window'
        self.left = 300
        self.top = 100
        self.width = 600
        self.height = 800

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.solenoid = SolenoidWidget(self)
        self.show()