
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


from CsvHelper import CsvHelper
from SolenoidHelper import SolenoidHelper
from TankHelper import TankHelper

from Solenoid import Solenoid

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

    solenoidList = []

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

        self.left = 0
        self.top = 0
        self.width = 1680
        self.height = 1050
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

        self.qp = QPainter()

        self.initHelpers()
        self.initConfigFiles()
        self.createObjects()


    def initConfigFiles(self):
        self.csvObjectData = self.csvHelper.loadCsv("csvObjectData.csv")


    def initHelpers(self):
        self.csvHelper = CsvHelper()
        self.tnkHelper = TankHelper()

    def createObjects(self):

        for i in range(self.csvObjectData[1]):
            # Creates horizontal and vertical solenoids
            if int(self.csvObjectData[2][0][i]) == 0 or int(self.csvObjectData[2][0][i]) == 1:
                solenoid = Solenoid(self, len(self.solenoidList),
                                    [float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])],
                                    int(self.csvObjectData[2][1][i]), int(self.csvObjectData[2][0][i]))
                self.solenoidList.append(solenoid)


    def paintEvent(self, e):
        self.qp.begin(self)
        self.qp.setRenderHint(QPainter.HighQualityAntialiasing)

        #Draw Solenoids
        for i in range(len(self.solenoidList)):
            self.solenoidList[i].draw()

        #self.solHelper.drawSolenoids2(self.qp, self.objectScale)
        self.tnkHelper.drawTank1(self.qp, self.objectScale, self.counter)

        self.qp.end()

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        quitAction = menu.addAction("Test RMB")
        action = menu.exec_(self.mapToGlobal(event.pos()))


