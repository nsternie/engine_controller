from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from csvHelper import CsvHelper
from constants import Constants
from solenoid import Solenoid
from tank import Tank
from pressureTransducer import PressureTransducer
from MathHelper import MathHelper
from controlsPanelWidget import  ControlsPanelWidget

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

        # Width of the panel on the right hand side
        # HMM: Should this go here or in the ControlsPanelWidget Class?
        self.panel_width = 300

        # Marker for if the controls area is being edited
        self.is_editing = False


        self.controlsWidget = ControlsWidget(self)
        self.controlsPanelWidget = ControlsPanelWidget(self)

        # Some variables depend on the init of ControlsPanelWidget so has to happen after it inits
        self.controlsWidget.finalizeInit()

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
        self.window = parent
        self.gui = self.window.parent

        self.left = 0
        self.top = 0

        self.width = self.gui.screenResolution[0] - self.window.panel_width
        self.height = self.gui.screenResolution[1]
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

        # Object Tracker
        self.object_list = []

        # Solenoid and tank trackers
        self.solenoid_list = []
        self.tank_list = []
        self.pressure_transducer_list = []

        # painter controls the drawing of everything on the widget
        self.painter = QPainter()

        self.initConfigFiles()
        self.createObjects()

        # Masa Logo on bottom left of screen
        # FIXME: Make this not blurry as hell
        # TODO: Move this to the main window instead of the widget
        # TODO: Make CustomMainWindow Class to handle things like this for all windows
        self.masa_logo = QLabel(self)
        pixmap = QPixmap('masawhiteworm.png')
        self.masa_logo.setPixmap(pixmap)
        self.masa_logo.setGeometry(10, self.gui.screenResolution[1] - 110, 300, 100)


    # TODO: Almost anything but this, that being said it works
    def finalizeInit(self):
        """
        There simply must be an elegant solution to this
        """
        self.controlsPanel = self.window.controlsPanelWidget

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
            self.csvObjectData[2][2][i] = MathHelper.mapValue(float(self.csvObjectData[2][2][i]), 139, 790, self.screenWidthBuffer, self.parent.parent.screenResolution[0]-self.screenWidthBuffer - 300)
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
                    Solenoid(self, QPointF(float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])),
                             int(self.csvObjectData[2][1][i]), int(self.csvObjectData[2][0][i])))
            # Creates Tanks
            if int(self.csvObjectData[2][0][i]) == 2:
                self.tank_list.append(
                    Tank(self, QPointF(float(self.csvObjectData[2][2][i]), float(self.csvObjectData[2][3][i])),
                          int(self.csvObjectData[2][1][i])))


        self.pressure_transducer_list.append(PressureTransducer(self, QPointF(20, 300), 0, False))

        self.edit_button = QPushButton("EDIT", self)
        self.edit_button.clicked.connect(lambda : self.toggleEdit())
        self.edit_button.move(self.gui.screenResolution[0]-self.window.panel_width - self.edit_button.width() - 30, 30)
        self.edit_button.show()


    def toggleEdit(self):
        """
        Toggles if the window is in edit mode or not
        """
        self.window.is_editing = not self.window.is_editing

        if self.window.is_editing:
            self.edit_button.setText("SAVE")
        else:
            self.edit_button.setText("EDIT")
            self.controlsPanel.edit_frame.hide()
            self.controlsPanel.save()

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

        for pt in self.pressure_transducer_list:
            pt.draw()

        self.painter.end()
