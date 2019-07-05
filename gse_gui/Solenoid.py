from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from Constants import Constants
from Tank1 import Tank1

class Solenoid(QPushButton):

    solenoidList = []

    #Beef of the setup for creating solenoid button object
    def __init__(self, widgetParent, position, fluid, isVertical):

        self.widgetParent = widgetParent # Important for getting sender

        #This ID could be calculated better to avoid repeats but it works for now
        self.id = len(self.solenoidList) #Very important! DO NOT CHANGE FROM WHAT PROGRAM SET

        #Should be grabbed by csv and scaled
        self.height = 18 * 1.75;
        self.width = 40 * 1.75;

        #These values will eventually be able to be edited by the user
        self.avionicsNumber = -1
        self.shortName = 'OX-SN-G07'
        self.state = 0
        self.safetyStatus = -1
        self.longName = 'LOX Dewar Drain'
        self.position = position
        self.fluid = fluid
        self.isVertical = isVertical

        if self.id == 16:
            self.labelPosition = 1
        else:
            self.labelPosition = 0


        #Create Button and style it
        button = QPushButton(self.widgetParent)
        button.setStyleSheet("background-color:transparent;border:0;")
        button.setContextMenuPolicy(Qt.DefaultContextMenu)
        button.setToolTip(self.shortName + "\nState: Closed")

        button.move(self.position[0],self.position[1])

        #If the sol is vertical set the button size accordingly
        if self.isVertical == 1:
            button.resize(self.height, self.width)
        else:
            button.resize(self.width,self.height)

        #Connect the button and show it
        button.clicked.connect(lambda: self.onClick())
        button.show()
        self.button = button


        #Label next to sol
        label = QLabel(self.widgetParent)

        #Get font and set it
        font = QFont()
        font.setStyleStrategy(QFont.PreferAntialias)
        font.setFamily("Arial")
        font.setPointSize(23)
        label.setFont(font)

        # Sets the sizing of the label
        label.setFixedWidth(self.width)
        label.setFixedHeight(80)  # 80 Corresponds to three rows at this font type and size (Arial 23)
        label.setText(self.longName)  # Solenoid long name
        label.setStyleSheet('color: white')
        label.setWordWrap(1)


        #This is a fucking mess but I am too hella lazy to fix it rn
        if self.labelPosition == 0:
            label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            if self.isVertical == 0:
                label.move(self.position[0], self.position[1] - label.height())
            else:
                label.move(self.position[0] - label.width() / 2 + self.height / 2, self.position[1] - label.height())
        elif self.labelPosition == 1:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.isVertical == 0:
                label.move(self.position[0] + self.width, self.position[1] - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position[0] + self.height, self.position[1] - label.height() / 2 + self.width / 2)
        elif self.labelPosition == 2:
            label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            if self.isVertical == 0:
                label.move(self.position[0], self.position[1] + self.height)
            else:
                label.move(self.position[0] - label.width() / 2 + self.height / 2, self.position[1] + self.width)
        elif self.labelPosition == 3:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.isVertical == 0:
                label.move(self.position[0] - self.width, self.position[1] - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position[0] - self.width, self.position[1] - label.height() / 2 + self.width / 2)

        label.show()

        self.label = label

        #Finnally and VERY importantly add the Solenoid to the big list
        self.solenoidList.append(self)

    def draw(self):

        path = QPainterPath()

        # To make coding easier
        xPos = self.position[0]
        yPos = self.position[1]

        # If solenoid is open color it in
        if self.state == 1:
            self.widgetParent.qp.setBrush(Constants.fluidColor[self.fluid])  # This function colors in a path
        else:
            self.widgetParent.qp.setBrush(0)

        # Need to update and grab real color from sol list
        self.widgetParent.qp.setPen(Constants.fluidColor[self.fluid])

        # Move path to starting position
        path.moveTo(xPos, yPos)  # Top left corner

        # = 0 -> Draw horizontally
        if self.isVertical == 0:
            path.lineTo(xPos, yPos + self.height)  # Straight Down
            path.lineTo(xPos + self.width, yPos)  # Diag to upper right
            path.lineTo(xPos + self.width, yPos + self.height)  # Straight Up
            path.lineTo(xPos, yPos)
        else:  # Draw vertically
            path.lineTo(xPos + self.height, yPos)
            path.lineTo(xPos, yPos + self.width)
            path.lineTo(xPos + self.height, yPos + self.width)
            path.lineTo(xPos, yPos)

        self.widgetParent.qp.drawPath(path)

        self.widgetParent.qp.fillRect(QRectF(self.position[0], self.position[1], 7, 7),
                                      Constants.fluidColor[self.fluid])

    def onClick(self):
        # Gets the senders(button) solenoidList index from the accessibleName
        #self.widgetParent.counter = self.widgetParent.counter + .05

        if self.id == 0:
            Tank1.tank1List[0].fillPercent = Tank1.tank1List[0].fillPercent + .05
        elif self.id == 1:
            Tank1.tank1List[1].fillPercent = Tank1.tank1List[1].fillPercent + .05
        elif self.id == 2:
            Tank1.tank1List[2].fillPercent = Tank1.tank1List[2].fillPercent + .05

        print(self.id)
        print(len(self.solenoidList))
        self.toggle()
        self.widgetParent.update()


    def move(self, xPos, yPos):
        self.button.move(xPos,yPos)
        self.position = [xPos, yPos]


    def toggle(self):
        if self.state == 0:
            self.state = 1
            self.button.setToolTip(self.shortName + "\nState: Open")
        elif self.state == 1:
            self.state = 0
            self.button.setToolTip(self.shortName + "\nState: Closed")
        else:
            print("WARNING STATE OF SOLENOID " + str(self.id) + " IS NOT PROPERLY DEFINED")



