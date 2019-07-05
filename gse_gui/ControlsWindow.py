
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from CsvHelper import CsvHelper

from Solenoid import Solenoid
from Tank1 import Tank1
from MathHelper import MathHelper

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

    screenWidthBuffer = 100
    screenHeightBuffer = 100
    objectScale = 1.75
    positionScale = 1
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

        self.qp = QPainter()

        self.initHelpers()
        self.initConfigFiles()
        self.createObjects()


    def initConfigFiles(self):
        self.csvObjectData = self.csvHelper.loadCsv("csvObjectData.csv")

        for i in range(self.csvObjectData[1]):
            print(self.csvObjectData[2][2][i] + " " + self.csvObjectData[2][3][i])
            self.csvObjectData[2][2][i] = MathHelper.mapValue(float(self.csvObjectData[2][2][i]), 139, 790, self.screenWidthBuffer, self.parent.parent.screenResolution[0]-self.screenWidthBuffer)
            self.csvObjectData[2][3][i] = MathHelper.mapValue(float(self.csvObjectData[2][3][i]), 179, 590, self.screenHeightBuffer,self.parent.parent.screenResolution[1] - self.screenHeightBuffer)
            print(str(self.csvObjectData[2][2][i]) + " A " + str(self.csvObjectData[2][3][i]))

    def initHelpers(self):
        self.csvHelper = CsvHelper()

    def createObjects(self):

        for i in range(self.csvObjectData[1]):
            # Creates horizontal and vertical solenoids
            if int(self.csvObjectData[2][0][i]) == 0 or int(self.csvObjectData[2][0][i]) == 1:
                Solenoid(self, [float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])], int(self.csvObjectData[2][1][i]), int(self.csvObjectData[2][0][i]))
            if int(self.csvObjectData[2][0][i]) == 2:
                Tank1(self, [float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])], int(self.csvObjectData[2][1][i]))


    def paintEvent(self, e):
        self.qp.begin(self)
        self.qp.setRenderHint(QPainter.HighQualityAntialiasing)

        #Draw Solenoids
        for solenoid in Solenoid.solenoidList:
            solenoid.draw()

        for tank1 in Tank1.tank1List:
            tank1.draw()

        #self.tnkHelper.drawTank1(self.qp, self.objectScale, self.counter)

        self.qp.end()

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        quitAction = menu.addAction("Test RMB")
        action = menu.exec_(self.mapToGlobal(event.pos()))

    def calculatePositionScale(self):
        print("tes")

