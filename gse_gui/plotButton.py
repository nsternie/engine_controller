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
            pos = event.globalPos() - self.window_pos - self.drag_start_pos

            # OPTIMIZE: Can create a list of all anchor points, sort it and then check the index before and after,
            # to determine if there is a object to snap too.

            # Determines if the object should be 'snapped' into place
            # Triple for loop :(. First for loop runs through all anchor points on the object, then for every object onscreen,
            # Checks to see if the current anchor point, is near another object anchor point
            for anchor_point in self.object_.anchor_points:
                anchor_point.x_aligned = False
                anchor_point.y_aligned = False
                for obj in self.parent.object_list:
                    if obj is not self.object_:
                        for obj_ap in obj.anchor_points:
                            if obj_ap.x() - 5 < anchor_point.x() < obj_ap.x() + 5 and abs(pos.x() - self.object_.position.x()) < 5:
                                pos = QPoint(obj_ap.x() + (self.object_.position.x() - anchor_point.x()), pos.y())
                                anchor_point.x_aligned = True
                            if obj_ap.y() - 5 < anchor_point.y() < obj_ap.y() + 5 and abs(pos.y() - self.object_.position.y()) < 5:
                                pos = QPoint(pos.x(), obj_ap.y() + (self.object_.position.y() - anchor_point.y()))
                                anchor_point.y_aligned = True


            self.object_.move(pos)

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




