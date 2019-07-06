from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants
from plot import PlotButton
from object import BaseObject

"""
Class to handle all solenoid objects and their functionality 
"""
class Solenoid(QPushButton, BaseObject):

    def __init__(self, widgetParent, position, fluid, isVertical):

        """
        Init the solenoid object
        :param widgetParent: widget this object will be added to
        :param position: position of icon on screen
        :param fluid: fluid in object
        :param isVertical: tracker if object is drawn vertically
        :return:
        """

        # Initialize base classes
        super().__init__(parent=widgetParent, position=position, fluid=fluid, is_vertical=isVertical)

        self.widgetParent = widgetParent # Important for drawing object

        # This ID could be calculated better to avoid repeats but it works for now
        self._id = len(self.widgetParent.solenoid_list) #Very important! DO NOT CHANGE FROM WHAT PROGRAM SET

        # Should be grabbed by csv and scaled
        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widgetParent
        self.height = 18 * 1.75
        self.width = 40 * 1.75

        # State tracks whether the solenoid is open or closed
        self.state = 0

        # Label Position designates the side of the object the label will be placed
        # TODO: Make dictionary to designate, top, bottom, left and right positions
        if self._id == 16:
            self.labelPosition = 1
        else:
            self.labelPosition = 0

        #Create Button and style it
        button = PlotButton(self.short_name, 'data.csv', 'Pressure', self.widgetParent)
        button.setStyleSheet("background-color:transparent;border:0;")
        button.setContextMenuPolicy(Qt.CustomContextMenu)
        button.setToolTip(self.short_name + "\nState: Closed")

        button.move(self.position[0], self.position[1])
        self.contextMenu = QMenu(self.widgetParent)
        # FIXME: Context menu always appears in the top left
        self.contextMenu.move(self.position[0], self.position[1])

        ## Add quitAction
        self.contextMenu.addAction("Test RMB")

        # If the sol is vertical set the button size accordingly
        # TODO: Update self.height and width if the solenoid is vertical instead of doing this
        if self.is_vertical == 1:
            button.resize(self.height, self.width)
        else:
            button.resize(self.width,self.height)

        # Connect plot options to button context menu
        button.clicked.connect(lambda: self.onClick())
        button.customContextMenuRequested.connect(
            lambda *args: self.plot_menu(*args, button, self.contextMenu)
        )
        button.show()
        self.button = button

        # Label next to sol
        label = QLabel(self.widgetParent)

        # Get font and set it
        font = QFont()
        font.setStyleStrategy(QFont.PreferAntialias)
        font.setFamily("Arial")
        font.setPointSize(23)
        label.setFont(font)

        # Sets the sizing of the label
        label.setFixedWidth(self.width)
        label.setFixedHeight(80)  # 80 Corresponds to three rows at this font type and size (Arial 23)
        label.setText(self.long_name)  # Solenoid long name
        label.setStyleSheet('color: white')
        label.setWordWrap(1)


        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            if self.is_vertical == 0:
                label.move(self.position[0], self.position[1] - label.height())
            else:
                label.move(self.position[0] - label.width() / 2 + self.height / 2, self.position[1] - label.height())
        elif self.labelPosition == 1:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                label.move(self.position[0] + self.width, self.position[1] - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position[0] + self.height, self.position[1] - label.height() / 2 + self.width / 2)
        elif self.labelPosition == 2:
            label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            if self.is_vertical == 0:
                label.move(self.position[0], self.position[1] + self.height)
            else:
                label.move(self.position[0] - label.width() / 2 + self.height / 2, self.position[1] + self.width)
        elif self.labelPosition == 3:
            label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                label.move(self.position[0] - self.width, self.position[1] - label.height() / 2 + self.height / 2)
            else:
                label.move(self.position[0] - self.width, self.position[1] - label.height() / 2 + self.width / 2)

        label.show()

        self.label = label

    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        #Holds the path of lines to draw
        path = QPainterPath()

        # To make coding easier
        xPos = self.position[0]
        yPos = self.position[1]

        # If solenoid is open color it in
        if self.state == 1:
            self.widgetParent.painter.setBrush(Constants.fluidColor[self.fluid])  # This function colors in a path
        else:
            self.widgetParent.painter.setBrush(0)

        # Sets line color
        self.widgetParent.painter.setPen(Constants.fluidColor[self.fluid])

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

        self.widgetParent.painter.drawPath(path)

        # This is debug, draws a box around the origin of object
        self.widgetParent.painter.fillRect(QRectF(self.position[0], self.position[1], 7, 7),
                                      Constants.fluidColor[self.fluid])

    def onClick(self):
        """
        When a solenoid is clicked this function is called
        """

        # This is for testing and will normally be used with capacitive level sensor
        if self._id < len(self.widgetParent.tank_list):
            self.widgetParent.tank_list[self._id].fillPercent += .05
        #Toggle state of solenoid
        self.toggle()
        #Tells widget painter to update screen
        self.widgetParent.update()

    def move(self, xPos, yPos):
        """
        Move solenoid to a new position

        :param xPos: new x position
        :param yPos: new y position
        """
        self.button.move(xPos, yPos)
        self.contextMenu.move(xPos, yPos)
        self.position = [xPos, yPos]

    def toggle(self):
        """
        Toggle the state of the solenoid
        """

        if self.state == 0:
            self.state = 1
            self.button.setToolTip(self.short_name + "\nState: Open")
        elif self.state == 1:
            self.state = 0
            self.button.setToolTip(self.short_name + "\nState: Closed")
        else:
            print("WARNING STATE OF SOLENOID " + str(self._id) + " IS NOT PROPERLY DEFINED")

    def plot_menu(self, event, button, menu):
        """
        Handler for context menu. These menus hand-off data plotting to plot windows
        :param event: default event from pyqt
        :param button: button instance this plot_menu is connected to
        :param menu: input QMenu object to display options on
        :return:
        """
        self.plotMenuActions = []
        for plot in self.widgetParent.parent.parent.plotWindow.plotList:
            action = QAction(plot.name)
            self.plotMenuActions.append(action)

            self.plotMenuActions[-1].triggered.connect(
                lambda *args, p=plot: self.link_plot(p, button)
            )

            menu.addAction(self.plotMenuActions[-1])

        menu.exec_(self.mapToGlobal(event))

    def link_plot(self, plot, button):
        """
        Link a Plot object to a given data file
        :param plot: plot object that needs a link to a data file
        :param button: button instance that was clicked on
        :return:
        """
        plot.link_data(button)