from PyQt5.QtGui import *
from PyQt5.QtCore import *

from overrides import overrides

from constants import Constants
from object import BaseObject


"""
Class to handle all solenoid objects and their functionality 
"""
class Solenoid(BaseObject):

    object_name = "Solenoid"
    def __init__(self, widget_parent, position, fluid, isVertical):

        """
        Init the solenoid object
        :param widget_parent: widget this object will be added to
        :param position: position of icon on screen
        :param fluid: fluid in object
        :param isVertical: tracker if object is drawn vertically
        :return:
        """

        # TODO: Still bleah, should have a way to rotate or something
        if isVertical:
            super().__init__(parent=widget_parent, position=position, fluid=fluid, width=18 * 1.75, height=40 * 1.75,name="Solenoid", is_vertical=isVertical, is_being_edited=False)
        else:
            # Initialize base classes
            super().__init__(parent=widget_parent, position=position, fluid=fluid, width= 40*1.75, height = 18*1.75, name = "Solenoid", is_vertical=isVertical, is_being_edited = False)

        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widget_parent

        # State tracks whether the solenoid is open or closed
        self.state = 0

        # Label Position designates the side of the object the label will be placed
        # TODO: Make dictionary to designate, top, bottom, left and right positions
        # This is so badddddd
        self.labelPosition = self.long_name_label_position_num

        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.width / 2, self.position.y() - self.long_name_label.height())
        elif self.labelPosition == 1:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            self.long_name_label.move(self.position.x() + self.width, self.position.y() - self.long_name_label.height() / 2 + self.height / 2)
        elif self.labelPosition == 2:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.width / 2, self.position.y() + self.height)
        elif self.labelPosition == 3:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            self.long_name_label.move(self.position.x() - self.height, self.position.y() - self.long_name_label.height() / 2 + self.height / 2)

    @overrides
    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        # Holds the path of lines to draw
        path = QPainterPath()

        # If solenoid is open color it in
        if self.state == 1:
            self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])  # This function colors in a path
        else:
            self.widget_parent.painter.setBrush(0)

        # Sets line color
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])

        # Move path to starting position
        path.moveTo(0, 0)  # Top left corner

        # = 0 -> Draw horizontally
        if self.is_vertical == 0:
            path.lineTo(0,self.height)  # Straight Down
            path.lineTo(self.width,0)  # Diag to upper right
            path.lineTo(self.width, self.height)  # Straight Up
            path.lineTo(0, 0)
        else:  # Draw vertically
            path.lineTo(self.width, 0)
            path.lineTo(0, self.height)
            path.lineTo(self.width, self.height)
            path.lineTo(0, 0)


        path.translate(self.position.x(), self.position.y())

        self.widget_parent.painter.drawPath(path)

        self.widget_parent.painter.setBrush(0)

        super().draw()

        # This is debug, draws a box around the origin of object
        #self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y(), 7, 7),Constants.fluidColor[self.fluid])

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
        elif self.state == 1:
            self.state = 0
            self.setToolTip_("State: Closed")
        else:
            print("WARNING STATE OF SOLENOID " + str(self._id) + " IS NOT PROPERLY DEFINED")

    # TODO: Get rid of this label position number stuff
    @overrides
    def setLongNameLabelPosition(self, label_num: int, label_position: QPoint = None):
        """
        Updates the label position value and then updates the labels position on screen
        """

        self.labelPosition = label_num
        self.long_name_label_position_num = label_num

        label = self.long_name_label
        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.width / 2,
                                      self.position.y() - self.long_name_label.height())
        elif self.labelPosition == 1:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            self.long_name_label.move(self.position.x() + self.width,
                                      self.position.y() - self.long_name_label.height() / 2 + self.height / 2)
        elif self.labelPosition == 2:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            self.long_name_label.move(self.position.x() - self.long_name_label.width() / 2 + self.width / 2,
                                      self.position.y() + self.height)
        elif self.labelPosition == 3:
            self.long_name_label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            self.long_name_label.move(self.position.x() - self.width,
                                      self.position.y() - self.long_name_label.height() / 2 + self.height / 2)
