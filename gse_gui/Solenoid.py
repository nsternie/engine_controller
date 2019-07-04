from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from PlotButton import PlotButton


class Solenoid(QPushButton):

    def __init__(self, widgetParent, idNumber, position, fluid, isVertical):

        self.widgetParent = widgetParent

        self.id = idNumber

        #Should be grabbed by csv and scaled
        self.height = 30;
        self.width = 75;

        self.avionicsNumber = -1
        self.shortName = 'OX-SN-G07'
        self.state = 0
        self.safetyStatus = -1
        self.longName = 'LOX Dewar Drain'
        self.position = position
        self.fluid = fluid
        self.isVertical = isVertical
        self.labelPosition = 2

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
        label.setWordWrap(1)


        #This is a fucking mess but too hella lazy to fix it rn
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

    def onClick(self):
        # Gets the senders(button) solenoidList index from the accessibleName
        self.widgetParent.counter = self.widgetParent.counter + .05
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


