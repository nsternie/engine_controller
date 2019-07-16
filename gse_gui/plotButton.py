from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from overrides import overrides


class PlotButton(QPushButton):
    """
    Button class that holds a data file
    """

    ## Allowed data types, including solenoid state (0 or 1)
    allowed_data_types = ['Force','Temperature','Pressure','State']
    def __init__(self, name, object_, dataFile, dataType, parent=None):
        """
        Init for PlotButton

        :param name: name on button
        :param object_: object button is assigned to
        :param dataFile: data file tied to button
        :param dataType: type of data in [Force, Temperature, Pressure, State]
        :param parent: parent window
        """
        super().__init__(name,parent)
        self.parent = parent
        self.name = name
        self.object_ = object_
        self.dataFile = dataFile
        self.dataType = dataType

        # Make sure button has no label
        self.setText("")

        assert dataType in self.allowed_data_types

    @overrides
    def mousePressEvent(self, event: QMouseEvent):
        """
        Called when mouse is pressed on a button. Used for drag and drop of objects

        :param event: variable holding event data
        """
        # If left click and the button is currently being edited
        if event.button() == Qt.LeftButton & self.object_.is_being_edited:
            # Set drag start position
            self.drag_start_pos = event.pos()

        super().mousePressEvent(event)

    @overrides
    def mouseMoveEvent(self, event: QMouseEvent):
        """
        Called when mouse is moving a button. Used for drag and drop of objects

        :param event: variable holding event data
        """
        if event.button() == Qt.LeftButton & self.object_.is_being_edited:
            # I have no idea where the 22 comes from
            # 22 is for non full screen on my (all?) macs
            # HMM: Elegant Solution?
            self.window_pos = self.parent.parent.pos() #+ QPoint(0, 22)

            # Sets that the object is currently being moved
            self.object_.is_being_dragged = True

            # Move the button into place on screen
            self.object_.move(event.globalPos() - self.window_pos - self.drag_start_pos)

        super().mouseMoveEvent(event)

    @overrides
    def mouseReleaseEvent(self, event: QMouseEvent):
        """
        Called when mouse releases a button. Used for drag and drop of objects

        :param event: variable holding event data
        """
        # Checks if the object is currently being dragged
        if event.button() == Qt.LeftButton & self.object_.is_being_edited & self.object_.is_being_dragged:

            # Does background stuff when object is released
            super().mouseReleaseEvent(event)

            # Makes sure that the button is still checked as editing and edit panel does not disappear
            self.parent.controlsPanel.addEditingObjects(self.object_)
            self.object_.is_being_dragged = False
        else:
            super().mouseReleaseEvent(event)




