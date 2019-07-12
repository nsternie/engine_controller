from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

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

        # painter controls the drawing of everything on the widget
        self.painter = QPainter()

        self.initConfigFiles()
        self.createObjects()

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

        self.painter.end()

class ControlsPanelWidget(QWidget):
    """
    Widget that contains controls that are not through icons on screen. Ex. Editing, Arming etc
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.window = parent
        # TODO: Rename the controls because it is weird
        self.controls = self.window.controlsWidget

        self.gui = self.window.parent

        # Keeps track of all the objects currently being edited
        self.objects_editing = []

        self.left = self.gui.screenResolution[0] - self.window.panel_width
        self.top = 0

        self.width = self.window.panel_width
        self.height = self.gui.screenResolution[1]
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Sets the color of the panel to dark Gray
        # TODO: Make this not look totally terrible
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkGray)
        self.setPalette(p)

        self.initEditFrame()

        self.show()

    def initEditFrame(self):
        """
        Inits the widgets inside of the editing frame
        """

        # Frame holds everything in it and can be hidden / shown
        self.edit_frame = QFrame(self)
        self.edit_form_layout = QFormLayout(self)

        long_name_label = QLabel("Long Name:")

        #Textbox to edit the longname of the object
        self.long_name_textbox = QLineEdit(self)
        self.long_name_textbox.setGeometry(20, 20, 200, 40)
        self.long_name_textbox.textChanged.connect(self.textChanged1)

        label_position_label = QLabel("Label Position:")

        # Textbox to edit the longname of the object
        self.label_position_textbox = QLineEdit(self)
        self.label_position_textbox.setGeometry(20, 20, 200, 40)
        self.label_position_textbox.textChanged.connect(self.textChanged2)
        self.label_position_textbox.setValidator(QIntValidator())

        # Add text boxes to the layout
        self.edit_form_layout.addRow(long_name_label, self.long_name_textbox)
        self.edit_form_layout.addRow(label_position_label, self.label_position_textbox)

        self.edit_frame.setLayout(self.edit_form_layout)
        self.edit_frame.hide()

    # FIXME: Things don't work well if more than one object are in here
    def addEditingObjects(self, objects):
        """
        Adds object to list to be edited
        """
        objects.is_being_edited = True
        self.objects_editing.append(objects)
        self.edit_frame.show()
        self.updateEditFields(objects)

    def removeEditingObjects(self, objects):
        """
        Removes object from list to be edited
        """
        objects.is_being_edited = False
        self.objects_editing.remove(objects)

        #If no objects are being edited hide the edit frame
        if len(self.objects_editing) == 0:
            self.edit_frame.hide()

    def removeAllEditingObjects(self):
        """
        Sets all objects to not be editing and clears the editing list
        """
        for object in self.objects_editing:
            object.is_being_edited = False

        self.objects_editing.clear()

    def updateEditFields(self, object):
        """
        Updates the various fields in the edit frame when a new object is selected for editing
        """
        self.long_name_textbox.setText(object.long_name)
        self.label_position_textbox.setText(str(object.labelPosition))

    def save(self):
        """
        This saves the edits and changes back into user mode
        """
        self.removeAllEditingObjects()

    # TODO: Make this work for multiple fields, not just one
    def textChanged1(self, text):
        """
        Called when the text of a text field is changed
        """

        #Currently for testing, sets the long_name and label text of solenoid
        for object in self.objects_editing:
            if object.is_being_edited:
                object.updateLongName(text)


    def textChanged2(self, text):
        """
        Called when the text of a text field is changed
        """
        #Currently for testing, sets the long_name and label text of solenoid
        # TODO: Change this to a drop down menu, with left, right etc options
        # FIXME: This does not work for tanks
        for sol in self.controls.solenoid_list:
            if sol.is_being_edited:
                if len(text) > 0:
                    sol.updateLabelPosition(int(text))





