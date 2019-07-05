from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants
from MathHelper import MathHelper

class Tank:

    # Beef of the setup for creating solenoid button object
    def __init__(self, widgetParent, position, fluid):
        self.widgetParent = widgetParent  # Important for getting sender

        # This ID could be calculated better to avoid repeats but it works for now
        self._id = len(self.widgetParent.tank_list)  # Very important! DO NOT CHANGE FROM WHAT PROGRAM SET

        # Should be grabbed by csv and scaled
        self.height = 170 * 1.75;
        self.width = 88 * 1.75;

        # These values will eventually be able to be edited by the user
        self.avionicsNumber = -1
        self.shortName = 'OX-SN-G07'
        self.safetyStatus = -1
        self.longName = 'LOX Dewar Drain'
        self.position = position
        self.fluid = fluid
        self.isVertical = 0
        self.fillPercent = 0

    def draw(self):

        arcHeight = 20

        # Draws the tank outline
        path = QPainterPath()
        self.widgetParent.qp.setPen(Constants.fluidColor[self.fluid])

        path.moveTo(self.position[0],self.position[1])
        path.arcTo(QRectF(self.position[0], self.position[1] - arcHeight, self.width, arcHeight * 2), 180, -180)
        path.lineTo(self.position[0] + self.width, self.position[1]+ self.height - 2 * arcHeight)
        path.arcTo(QRectF(self.position[0] + self.width, self.position[1] + self.height - arcHeight - 2 * arcHeight, - self.width, arcHeight * 2), 180, 180)
        path.lineTo(self.position[0], self.position[1])

        self.widgetParent.qp.drawPath(path)

        self.widgetParent.qp.fillRect(QRectF(self.position[0], self.position[1], 7, 7), Constants.fluidColor[self.fluid])

        # End tank outline draw

        # Fill in bottom arc
        path = QPainterPath()
        self.widgetParent.qp.setPen(Constants.fluidColor[self.fluid])
        self.widgetParent.qp.setBrush(Constants.fluidColor[self.fluid])

        # Maps the fill percentage of the tank to an angle to fill the bottom arc
        bottomArcFillAngle = MathHelper.mapValue(self.fillPercent, 0, arcHeight / self.height, 0, 90)

        path.moveTo(self.position[0] + self.width / 2, self.position[1] + self.height - arcHeight)
        path.arcTo(QRectF(self.position[0], self.position[1] + self.height - arcHeight - 2 * arcHeight, self.width, arcHeight * 2), 270, bottomArcFillAngle)
        path.lineTo(self.position[0] + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position[0] + self.width / 2, self.position[1] + self.height - arcHeight)

        path.moveTo(self.position[0] + self.width / 2, self.position[1] + self.height - arcHeight)
        path.arcTo(QRectF(self.position[0], self.position[1] + self.height - arcHeight - 2 * arcHeight, self.width, 2 * arcHeight), 270,
                   -bottomArcFillAngle)
        path.lineTo(self.position[0] + self.width / 2, path.currentPosition().y())
        path.lineTo(self.position[0] + self.width / 2, self.position[1] + self.height - arcHeight)

        self.widgetParent.qp.drawPath(path)
        # End fill in bottom arc

        # Fill in tank body
        # Maps fill percentage to the height of the body to fill
        bodyFillHeight = MathHelper.mapValue(self.fillPercent, arcHeight / self.height, 1 - arcHeight / self.height, 0,
                                       self.height - 2 * arcHeight)

        self.widgetParent.qp.fillRect(QRectF(self.position[0], self.position[1] - 2 * arcHeight + self.height - bodyFillHeight, self.width, bodyFillHeight), Constants.fluidColor[self.fluid])
        # End fill in tank body

        # Fill in top arc
        path = QPainterPath()
        self.widgetParent.qp.setPen(Constants.fluidColor[self.fluid])
        self.widgetParent.qp.setBrush(Constants.fluidColor[self.fluid])

        topArcFillAngle = MathHelper.mapValue(self.fillPercent, 1 - (arcHeight / self.height), 1, 0, 90)

        path.moveTo(self.position[0] + self.width, self.position[1])
        path.arcTo(QRectF(self.position[0], self.position[1] - arcHeight, self.width, arcHeight * 2), 0, topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position[0] + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position[0] + self.width / 2, self.position[1])

        path.moveTo(self.position[0], self.position[1])
        path.arcTo(QRectF(self.position[0], self.position[1] - arcHeight, self.width, arcHeight * 2), 180, -topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(self.position[0] + self.width / 2, path.currentPosition().y())
            path.lineTo(self.position[0] + self.width / 2, self.position[1])

        self.widgetParent.qp.drawPath(path)

        self.widgetParent.qp.setBrush(0)

        # End fill in top arc
