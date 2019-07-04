from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from PlotButton import PlotButton
from Solenoid import Solenoid
from Constants import Constants

class SolenoidHelper:
    solXPosList = []
    solYPosList = []
    solenoidList = [[0, 'OX-SN-G01', 0, 0, [1163, 100], 'LOX Main Fill', [0, 0]],
                    [1, 'OX-SN-G02', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [2, 'OX-SN-G03', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [3, 'OX-SN-G4', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [4, 'OX-SN-G05', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [5, 'OX-SN-G06', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [6, 'OX-SN-G07', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [7, 'OX-SN-G08', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [8, 'OX-SN-G09', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [9, 'OX-SN-G10', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [10, 'OX-SN-G11', 0, 0, [100, 178], 'LOX Dewar Drain', [0, 0]],
                    [11, 'OX-SN-G12', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [12, 'OX-SN-G13', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [13, 'OX-SN-G14', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [14, 'OX-SN-G15', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [15, 'OX-SN-G16', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [16, 'OX-SN-G17', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]],
                    [17, 'OX-SN-G18', 0, 0, [100, 178], 'LOX Dewar Drain', [1, 0]]]

    solenoidList2 = []

    # ****THESE SHOULD BE REORDERED******

    # Index 0: Button Reference (TBA later in program)
    # Index 1: Avionics Number
    # Index 2: Short Name
    # Index 3: State (0 = closed, 1 = open)
    # Index 4: Safety Status (0 = None, 1 = Warning, 2 = Critical)
    # Index 5: Position(LIST) [Xpos, Ypos]  ***WILL BE AUTO GENERATED FROM MATLAB****
    # Index 6: Long Name
    # Index 7: Solenoid & Label Positioning Data(LIST) [Solenoid Orientation, Label Relative Pos] (Sol Orientation: 0 = Horizontal, 1 = Vertical
    # Label Relative Pos: 0 = Above Solenoid, 1 = Right, 2 = Below, 3 = Left) ***Yes I know this is a stupid way of doing this***
    # Index 8: Label Reference


    def createObjects(self, controlsWidget, csvObjectData, csvObjectData2):


        self.createSolenoids(controlsWidget, csvObjectData2)




    def createSolenoids(self, controlsWidget, csvObjectData):

        for i in range(csvObjectData[1]):
            if int(csvObjectData[2][0][i]) == 0 or int(csvObjectData[2][0][i]) == 1:
                solenoid = Solenoid(controlsWidget, len(self.solenoidList2), [float(csvObjectData[2][2][i]), float(csvObjectData[2][3][i])], int(csvObjectData[2][1][i]), int(csvObjectData[2][0][i]))
                self.solenoidList2.append(solenoid)


    def drawSolenoids2(self, qp, objScale):
        # Solenoid can be drawn with just lines or filled in.
        # Filled in represents it open, just an outline is closed.

        # These values assume if the solenoid is drawn horizontally
        # Will eventually be grabbed from matlab csv data
        height = 18 * objScale;
        width = 40 * objScale;

        for i in range(len(self.solenoidList2)):
            # Define a new paint path
            path = QPainterPath()

            # To make coding easier
            xPos = self.solenoidList2[i].position[0]
            yPos = self.solenoidList2[i].position[1]

            # If solenoid is open color it in
            if self.solenoidList2[i].state == 1:
                qp.setBrush(Constants.fluidColor[self.solenoidList2[i].fluid]) #This function colors in a path
            else:
                qp.setBrush(0)

            # Need to update and grab real color from sol list
            qp.setPen(Constants.fluidColor[self.solenoidList2[i].fluid])

            #Move path to starting position
            path.moveTo(xPos, yPos) #Top left corner

            # = 0 -> Draw horizontally
            if self.solenoidList2[i].isVertical == 0:
                path.lineTo(xPos, yPos + height)        # Straight Down
                path.lineTo(xPos + width, yPos)         # Diag to upper right
                path.lineTo(xPos + width, yPos + height)# Straight Up
                path.lineTo(xPos, yPos)
            else: # Draw verticaly
                path.lineTo(xPos + height, yPos)
                path.lineTo(xPos, yPos + width)
                path.lineTo(xPos + height, yPos + width)
                path.lineTo(xPos, yPos)

            qp.drawPath(path)







