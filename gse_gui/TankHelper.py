from PyQt5.QtGui import *
from PyQt5.QtCore import *


class TankHelper:

    # When we actually have objects to create
    # def createObjects(self, controlsWidget, csvObjectData):
        # self.updateTankPositions(csvObjectData)



    # When we need labels
    # def createTankLabels(self, controlsWidget, csvObjectData):

    # When switchover to new csv file occure
    # def updateTankPositions(self, csvObjectData):


    def drawTank1(self, qp, objScale, fillPrecent):
        height = 170 * objScale;
        width = 88 * objScale;

        path = QPainterPath()
        qp.setPen(Qt.cyan)

        path.moveTo(120, 210)
        path.lineTo(120, 210 + height)
        qp.drawPath(path)
        path = QPainterPath()
        path.moveTo(120, 210 + height)

        qp.setBrush(Qt.cyan)
        path.arcTo(QRectF(120, 190 + height, width, 40), 180, 180)
        qp.drawPath(path)
        qp.setBrush(0)

        path = QPainterPath()
        path.moveTo(120 + width, 210 + height)
        path.lineTo(120 + width, 210)
        path.arcTo(QRectF(120, 190, width, 40), 0, 180)
        qp.drawPath(path)

        qp.fillRect(QRectF(120, height * (1 - fillPrecent) + 210, width, height * fillPrecent), Qt.cyan)



