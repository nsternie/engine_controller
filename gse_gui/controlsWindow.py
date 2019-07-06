
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from csvHelper import CsvHelper
from solenoid import Solenoid
from tank import Tank
from MathHelper import MathHelper

"""
This file contains the class to create the window and widget
"""

class ControlsWindow(QMainWindow):
    """
    Window to house the controlsWidget
    """

    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        # Set geometry
        self.title = 'Controls Window'

        # Below numbers are arbitrary
        # TODO: Make them not arbitrary
        self.left = 300
        self.top = 100
        self.width = 600
        self.height = 800

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.controlsWidget = ControlsWidget(self)
        self.show()



class ControlsWidget(QWidget):
    """
    Class that creates the Control Widget. This widget contains all the graphical representations of objects, that can
    also be interacted with
    """

    # Temp values
    # TODO: Properly implement these values
    screenWidthBuffer = 100
    screenHeightBuffer = 100
    objectScale = 1.75 # This maybe should be a instance variable of all objects

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

        self.left = 0
        self.top = 0
        # TODO: Get rid of this whole .parent.parent thing. Values like this should be more easily accessible
        self.width = self.parent.parent.screenResolution[0]
        self.height = self.parent.parent.screenResolution[1]
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

        # Solenoid and tank trackers
        self.solenoid_list = []
        self.tank_list = []

        # painter controls the drawing of everything on the widget
        self.painter = QPainter()

        self.initConfigFiles()
        self.createObjects()

    def initConfigFiles(self):
        """
        Loads in all the configuration files for the widget
        """

        self.csvObjectData = CsvHelper.loadCsv("csvObjectData.csv")

        # Map objects on screen to the screen resolution of the users screen.
        # This is a quick fix, this method messes with two objects that should be aligned
        # TODO: Change position scaling algorithm in such a way that maintains inter-object alignment
        # TODO: Move this loop somewhere else
        for i in range(self.csvObjectData[1]):
            self.csvObjectData[2][2][i] = MathHelper.mapValue(float(self.csvObjectData[2][2][i]), 139, 790, self.screenWidthBuffer, self.parent.parent.screenResolution[0]-self.screenWidthBuffer)
            self.csvObjectData[2][3][i] = MathHelper.mapValue(float(self.csvObjectData[2][3][i]), 179, 590, self.screenHeightBuffer,self.parent.parent.screenResolution[1] - self.screenHeightBuffer)

    def createObjects(self):
        """
        Creates objects to be drawn onscreen. Uses csv file to load in all the object types, positions, and fluids
        """
        # Loops through all the cols of the csv

        for i in range(self.csvObjectData[1]):
            # Creates horizontal and vertical solenoids
            if int(self.csvObjectData[2][0][i]) == 0 or int(self.csvObjectData[2][0][i]) == 1:
                self.solenoid_list.append(
                    Solenoid(self, [float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])],
                             int(self.csvObjectData[2][1][i]), int(self.csvObjectData[2][0][i])))
            # Creates Tanks
            if int(self.csvObjectData[2][0][i]) == 2:
                self.tank_list.append(
                    Tank(self, [float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])],
                          int(self.csvObjectData[2][1][i])))


    def paintEvent(self, e):
        """
        This event is called automatically in the background by pyQt. It is used to update the drawing on screen
        This function calls the objects own drawing methods to perform the actual drawing calculations
        """
        self.painter.begin(self)

        # This makes the objects onscreen more crisp
        self.painter.setRenderHint(QPainter.HighQualityAntialiasing)

        # Draw Solenoids
        for solenoid in self.solenoid_list:
            solenoid.draw()
        # Draw Tanks
        for tank in self.tank_list:
            tank.draw()

        self.painter.end()


