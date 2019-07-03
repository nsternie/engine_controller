from PyQt5.QtGui import *
from PyQt5.QtCore import *


class TankHelper:

    # When we actually have objects to create
    # def createObjects(self, controlsWidget, csvObjectData):
        # self.updateTankPositions(csvObjectData)



    # When we need labels
    # def createTankLabels(self, controlsWidget, csvObjectData):

    # When switchover to new csv file occur
    # def updateTankPositions(self, csvObjectData):



    def mapValue(self,value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin


        if value > leftMin and value < leftMax:
            # Convert the left range into a 0-1 range (float)
            valueScaled = float(value - leftMin) / float(leftSpan)
        elif value >= leftMax:
            valueScaled = 1
        elif value <= leftMin:
            valueScaled = 0

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)


    def drawTank1(self, qp, objScale, fillPrecent):
        # Height and width will be calculated later from csv
        height = 170 * objScale;
        width = 88 * objScale;

        arcHeight = 20


        # Draws the tank outline
        path = QPainterPath()
        qp.setPen(Qt.cyan)

        path.moveTo(150,210)
        path.arcTo(QRectF(150, 210 - arcHeight, width, arcHeight * 2), 180, -180)
        path.lineTo(150 + width, 210 + height - 2*arcHeight)
        path.arcTo(QRectF(150 + width, 210 + height - arcHeight - 2*arcHeight, -width, arcHeight * 2), 180, 180)
        path.lineTo(150, 210)

        qp.drawPath(path)

        # End tank outline draw

        # Fill in bottom arc
        path = QPainterPath()
        qp.setPen(Qt.cyan)
        qp.setBrush(Qt.cyan)

        # Maps the fill percentage of the tank to an angle to fill the bottom arc
        bottomArcFillAngle = self.mapValue(fillPrecent, 0, arcHeight / height, 0, 90)

        path.moveTo(150 + width/2, 210 + height - arcHeight)
        path.arcTo(QRectF(150, 210 + height - arcHeight - 2*arcHeight, width, arcHeight * 2), 270, bottomArcFillAngle)
        path.lineTo(150 + width/2, path.currentPosition().y())
        path.lineTo(150 + width/2, 210+height - arcHeight)

        path.moveTo(150 + width / 2, 210 + height - arcHeight)
        path.arcTo(QRectF(150, 210 + height - arcHeight - 2*arcHeight, width, 2 * arcHeight), 270, -bottomArcFillAngle)
        path.lineTo(150 + width / 2, path.currentPosition().y())
        path.lineTo(150 + width / 2, 210 + height - arcHeight)

        qp.drawPath(path)
        # End fill in bottom arc


        # Fill in tank body
        # Maps fill percentage to the height of the body to fill
        bodyFillHeight = self.mapValue(fillPrecent, arcHeight/height, 1 - arcHeight/height, 0, height - 2*arcHeight)

        qp.fillRect(QRectF(150, 210 - 2 * arcHeight + height - bodyFillHeight, width, bodyFillHeight), Qt.cyan)
        # End fill in tank body

        #Fill in top arc
        path = QPainterPath()
        qp.setPen(Qt.cyan)
        qp.setBrush(Qt.cyan)

        topArcFillAngle = self.mapValue(fillPrecent, 1 - (arcHeight/height), 1, 0, 90)

        path.moveTo(150 + width, 210)
        path.arcTo(QRectF(150, 210 - arcHeight, width, arcHeight * 2), 0, topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(150+width/2, path.currentPosition().y())
            path.lineTo(150 + width/2, 210)

        path.moveTo(150, 210)
        path.arcTo(QRectF(150, 210 - arcHeight, width, arcHeight * 2), 180, -topArcFillAngle)
        if topArcFillAngle > 0:
            path.lineTo(150 + width / 2, path.currentPosition().y())
            path.lineTo(150 + width / 2, 210)

        qp.drawPath(path)

        #End fill in top arc



