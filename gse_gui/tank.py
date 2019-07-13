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
        super().__init__(parent=widget_parent, position=position, fluid=fluid, width= 88*1.75, height = 170*1.75, is_vertical=False, is_being_edited = False)

        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widget_parent

        # Tracks the percentage of fluid in the tank
        self.fillPercent = 0

        # Holder for right now
        # TODO: Implement the rest of this
        self.labelPosition = 0

        self.button.move(self.position.x(), self.position.y()-20)

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

    @overrides
    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        # Height of curved arc at top and bottom of tank
        arcHeight = 20

        # Draws the tank outline
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])

        path.moveTo(self.position.x(),self.position.y())
        path.arcTo(QRectF(self.position.x(), self.position.y() - arcHeight, self.width, arcHeight * 2), 180, -180)
        path.lineTo(self.position.x() + self.width, self.position.y()+ self.height - 2 * arcHeight)
        path.arcTo(QRectF(self.position.x() + self.width, self.position.y() + self.height - arcHeight - 2 * arcHeight, - self.width, arcHeight * 2), 180, 180)
        path.lineTo(self.position.x(), self.position.y())

        self.widget_parent.painter.drawPath(path)

        self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y(), 10, 10), Constants.fluidColor[self.fluid])

        # End tank outline draw

        # Fill in bottom arc
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])

        # Maps the fill percentage of the tank to an angle to fill the bottom arc
        bottomArcFillAngle = MathHelper.mapValue(self.fillPercent, 0, arcHeight / self.height, 0, 90)

        path.moveTo(self.position.x() + self.width / 2, self.position.y() + self.height - arcHeight)
        path.arcTo(QRectF(self.position.x(), self.position.y() + self.height - arcHeight - 2 * arcHeight, self.width, arcHeight * 2), 270, bottomArcFillAngle)
        path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position.x() + self.width / 2, self.position.y() + self.height - arcHeight)

        path.moveTo(self.position.x() + self.width / 2, self.position.y() + self.height - arcHeight)
        path.arcTo(QRectF(self.position.x(), self.position.y() + self.height - arcHeight - 2 * arcHeight, self.width, 2 * arcHeight), 270,
                   -bottomArcFillAngle)
        path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position.x() + self.width / 2, self.position.y() + self.height - arcHeight)

        self.widget_parent.painter.drawPath(path)
        # End fill in bottom arc

        # Fill in tank body
        # Maps fill percentage to the height of the body to fill
        bodyFillHeight = MathHelper.mapValue(self.fillPercent, arcHeight / self.height, 1 - arcHeight / self.height, 0,
                                       self.height - 2 * arcHeight)

        self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y() - 2 * arcHeight + self.height - bodyFillHeight, self.width, bodyFillHeight), Constants.fluidColor[self.fluid])
        # End fill in tank body

        # Fill in top arc
        path = QPainterPath()
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])

        topArcFillAngle = MathHelper.mapValue(self.fillPercent, 1 - (arcHeight / self.height), 1, 0, 90)

        path.moveTo(self.position.x() + self.width, self.position.y())
        path.arcTo(QRectF(self.position.x(), self.position.y() - arcHeight, self.width, arcHeight * 2), 0, topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position.x() + self.width / 2, self.position.y())

        path.moveTo(self.position.x(), self.position.y())
        path.arcTo(QRectF(self.position.x(), self.position.y() - arcHeight, self.width, arcHeight * 2), 180, -topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position.x() + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position.x() + self.width / 2, self.position.y())

        self.widget_parent.painter.drawPath(path)

        self.widget_parent.painter.setBrush(0)

        # End fill in top arc
