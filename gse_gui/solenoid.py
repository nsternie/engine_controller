from PyQt5.QtGui import *
from PyQt5.QtCore import *

from overrides import overrides

from constants import Constants
from object import BaseObject


"""
Class to handle all solenoid objects and their functionality 
"""
class Solenoid(BaseObject):

    def __init__(self, widget_parent, position, fluid, isVertical):

        """
        Init the solenoid object
        :param widget_parent: widget this object will be added to
        :param position: position of icon on screen
        :param fluid: fluid in object
        :param isVertical: tracker if object is drawn vertically
        :return:
        """

        # Initialize base classes
        super().__init__(parent=widget_parent, position=position, fluid=fluid, width= 40*1.75, height = 18*1.75, is_vertical=isVertical, is_being_edited = False)

        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widget_parent

        # State tracks whether the solenoid is open or closed
        self.state = 0

        # Label Position designates the side of the object the label will be placed
        # TODO: Make dictionary to designate, top, bottom, left and right positions
        self.labelPosition = 0

        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            if self.is_vertical == 0:
                self.long_name_label.move(self.position.x(), self.position.y() - self.long_name_label.height())
            else:
                self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.height / 2, self.position.y() - self.long_name_label.height())
        elif self.labelPosition == 1:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                self.long_name_label.move(self.position.x() + self.width, self.position.y() - self.long_name_label.height() / 2 + self.height / 2)
            else:
                self.long_name_label.move(self.position.x() + self.height, self.position.y() - self.long_name_label.height() / 2 + self.width / 2)
        elif self.labelPosition == 2:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            if self.is_vertical == 0:
                self.long_name_label.move(self.position.x(), self.position.y() + self.height)
            else:
                self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.height / 2, self.position.y() + self.width)
        elif self.labelPosition == 3:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                self.long_name_label.move(self.position.x() - self.width, self.position.y() - self.long_name_label.height() / 2 + self.height / 2)
            else:
                self.long_name_label.move(self.position.x() - self.width, self.position.y() - self.long_name_label.height() / 2 + self.width / 2)

    @overrides
    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        #Holds the path of lines to draw
        path = QPainterPath()

        # To make coding easier
        xPos = self.position.x()
        yPos = self.position.y()

        # If solenoid is open color it in
        if self.state == 1:
            self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])  # This function colors in a path
        else:
            self.widget_parent.painter.setBrush(0)

        # Sets line color
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])

        # Move path to starting position
        path.moveTo(xPos, yPos)  # Top left corner

        # = 0 -> Draw horizontally
        if self.is_vertical == 0:
            path.lineTo(xPos, yPos + self.height)  # Straight Down
            path.lineTo(xPos + self.width, yPos)  # Diag to upper right
            path.lineTo(xPos + self.width, yPos + self.height)  # Straight Up
            path.lineTo(xPos, yPos)
        else:  # Draw vertically
            path.lineTo(xPos + self.height, yPos)
            path.lineTo(xPos, yPos + self.width)
            path.lineTo(xPos + self.height, yPos + self.width)
            path.lineTo(xPos, yPos)

        self.widget_parent.painter.drawPath(path)

        # This is debug, draws a box around the origin of object
        self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y(), 7, 7),
                                      Constants.fluidColor[self.fluid])

    @overrides
    def onClick(self):
        """
        When a solenoid is clicked this function is called
        """

        super().onClick()

        if not self.widget_parent.window.is_editing:
            #Toggle state of solenoid
            self.toggle()

        # Tells widget painter to update screen
        self.widget_parent.update()

    def toggle(self):
        """
        Toggle the state of the solenoid
        """

        if self.state == 0:
            self.state = 1
            self.setToolTip_("State: Open")
            print("Test")
        elif self.state == 1:
            self.state = 0
            self.setToolTip_("State: Closed")
            print("test")
        else:
            print("WARNING STATE OF SOLENOID " + str(self._id) + " IS NOT PROPERLY DEFINED")


    def updateLabelPosition(self, position):
        """
        Updates the label position value and then updates the labels position on screen
        """
        self.labelPosition = position

        label = self.long_name_label
        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            if self.is_vertical == 0:
                label.move(self.position.x(), self.position.y() - label.height())
            else:
                label.move(self.position.x() - label.width() / 2 + self.height / 2, self.position.y() - label.height())
        elif self.labelPosition == 1:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                label.move(self.position.x() + self.width, self.position.y() - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position.x() + self.height, self.position.y() - label.height() / 2 + self.width / 2)
        elif self.labelPosition == 2:
            label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            if self.is_vertical == 0:
                label.move(self.position.x(), self.position.y() + self.height)
            else:
                label.move(self.position.x() - label.width() / 2 + self.height / 2, self.position.y() + self.width)
        elif self.labelPosition == 3:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                label.move(self.position.x() - self.width, self.position.y() - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position.x() - self.width, self.position.y() - label.height() / 2 + self.width / 2)

