from PyQt5.QtGui import *
from PyQt5.QtCore import *

from overrides import overrides

from constants import Constants
from MathHelper import MathHelper
from object import BaseObject


"""
Class to handle all tank objects and their functionality 
"""

# TODO: Tanks need to be more similar to solenoids so the base object can be expanded
class Tank(BaseObject):

    def __init__(self, widget_parent, position, fluid):

        """
        Init the solenoid object
        :param widget_parent: widget this object will be added to
        :param position: position of icon on screen. Passed as QPointF()
        :param fluid: fluid in object
        :return:
        """

        ## Initialize underlying class
        super().__init__(parent=widget_parent, position=position, fluid=fluid, width= 88*1.75, height = 170*1.75, name = "Tank", is_vertical=True, is_being_edited = False)

        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widget_parent

        # Tracks the percentage of fluid in the tank
        self.fillPercent = 0

        # Here is where some things done by the super init is re-done for tank specific objects
        self.short_name_label.setIsVertical(False)
        self.short_name_label.moveToPosition("Bottom")

        # TODO: Make this a better system
        self.long_name_label.setFixedWidth(self.width)
        self.setLongNameLabelPosition(self.long_name_label_position_num)


    @overrides
    def onClick(self):
        """
        When a tank is clicked this function is called
        This is really only useful for adding plots and
        selecting the tank for editing
        """
        super().onClick()

        if not self.widget_parent.window.is_editing:
            # This is for testing and will normally be used with capacitive level sensor
            self.fillPercent += .05

        # Tells widget painter to update screen
        self.widget_parent.update()

    def setLongNameLabelPosition(self, label_num: int, label_position: QPoint = None):
        """
        Sets the position of the long name label on an object
        :param label_num: num position of label -> Want to deprecate
        :param label_position: new position of label
        """
        self.long_name_label_position_num = label_num

        self.long_name_label.move(QPoint(int(self.position.x() + self.width /2 - self.long_name_label.width() / 2), self.position.y() + 20))

    @overrides
    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        # Height of curved arc at top and bottom of tank
        arcHeight = 20 * self.scale

        # Draws the tank outline
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])

        path.moveTo(0,arcHeight)
        path.arcTo(QRectF(0, 0, self.width, arcHeight * 2), 180, -180) # Top Arc
        path.lineTo(self.width, self.height - 2 * arcHeight) # Line down
        path.arcTo(QRectF(self.width, path.currentPosition().y(), - self.width, arcHeight * 2), 180, 180) # Bottom Arc
        path.lineTo(0, arcHeight) # Line up

        path.translate(self.position.x(), self.position.y()) # Translate it into position

        self.widget_parent.painter.drawPath(path) # Draw Path

        # Debug randomness
        #self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y(), 10, 10), Constants.fluidColor[self.fluid])

        # End tank outline draw

        # TODO: Make this a bit more basic to understand / helper function for filling an arc
        # Fill in bottom arc
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])

        # Maps the fill percentage of the tank to an angle to fill the bottom arc
        bottomArcFillAngle = MathHelper.mapValue(self.fillPercent, 0, arcHeight / self.height, 0, 90)

        path.moveTo(self.position.x() + self.width / 2, self.position.y() + self.height)
        path.arcTo(QRectF(self.position.x(), self.position.y() + self.height - 2 * arcHeight, self.width, arcHeight * 2), 270, bottomArcFillAngle)
        path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position.x() + self.width / 2, self.position.y() + self.height)

        path.moveTo(self.position.x() + self.width / 2, self.position.y() + self.height)
        path.arcTo(QRectF(self.position.x(), self.position.y() + self.height - 2 * arcHeight, self.width, 2 * arcHeight), 270,
                   -bottomArcFillAngle)
        path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position.x() + self.width / 2, self.position.y() + self.height)

        self.widget_parent.painter.drawPath(path)
        # End fill in bottom arc


        # Fill in tank body
        # Maps fill percentage to the height of the body to fill
        bodyFillHeight = MathHelper.mapValue(self.fillPercent, arcHeight / self.height, 1 - arcHeight / self.height, 0,
                                       self.height - 2 * arcHeight)

        self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y() - arcHeight + self.height - bodyFillHeight, self.width, bodyFillHeight), Constants.fluidColor[self.fluid])
        # End fill in tank body

        # Fill in top arc
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])

        topArcFillAngle = MathHelper.mapValue(self.fillPercent, 1 - (arcHeight / self.height), 1, 0, 90)

        path.moveTo(self.position.x() + self.width, self.position.y() + arcHeight)
        path.arcTo(QRectF(self.position.x(), self.position.y(), self.width, arcHeight * 2), 0, topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position.x() + self.width / 2, self.position.y() + arcHeight)

        path.moveTo(self.position.x(), self.position.y() + arcHeight)
        path.arcTo(QRectF(self.position.x(), self.position.y(), self.width, arcHeight * 2), 180, -topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position.x() + self.width / 2, self.position.y() + arcHeight)

        self.widget_parent.painter.drawPath(path)

        # self.widget_parent.painter.eraseRect(
        #     QRect(self.long_name_label.pos().x(), self.long_name_label.pos().y(), self.long_name_label.width(),
        #           self.long_name_label.height()))

        self.widget_parent.painter.setBrush(0)
        # End fill in top arc

        super().draw()
