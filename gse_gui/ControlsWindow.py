
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


from CsvHelper import CsvHelper
from SolenoidHelper import SolenoidHelper
from TankHelper import TankHelper

"""
    This file contains the class to create the window and widget
    """

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

        self.controlsWidget = ControlsWidget(self)
        self.show()



class ControlsWidget(QWidget):

    objectScale = 1.75

    counter = 0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

        self.left = 0
        self.top = 0
        self.width = 1680
        self.height = 1050
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

        self.initHelpers()
        self.initConfigFiles()
        self.createObjects()


    def initConfigFiles(self):
        self.csvObjectData = self.csvHelper.loadCsv("Sol.csv")


    def initHelpers(self):
        self.csvHelper = CsvHelper()
        self.solHelper = SolenoidHelper()
        self.tnkHelper = TankHelper()

    def createObjects(self):
        self.solHelper.createObjects(self, self.csvObjectData)


    qp = QPainter()

    def paintEvent(self, e):
        self.qp.begin(self)
        self.qp.setRenderHint(QPainter.HighQualityAntialiasing)

        self.solHelper.drawSolenoids(self.qp, self.objectScale)
        self.tnkHelper.drawTank1(self.qp, self.objectScale, self.counter)

        self.qp.end()

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        quitAction = menu.addAction("Test RMB")
        action = menu.exec_(self.mapToGlobal(event.pos()))

    def on_click(self):
        # Gets the senders(button) solenoidList index from the accessibleName
        self.counter = self.counter + .005
        index = int(self.sender().accessibleName())
        print(index)
        self.solHelper.toggleSolenoid(index)
        self.update()
