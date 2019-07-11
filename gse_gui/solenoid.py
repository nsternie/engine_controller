from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants
from plot import PlotButton
from object import BaseObject

"""
Class to handle all solenoid objects and their functionality 
"""
class Solenoid(BaseObject):

    def __init__(self, widget_parent, position, fluid, isVertical):

        """
        Init the solenoid object
        :param widget_parent: widget this object will be added to
        :param position: position of icon on screen
        :param fluid: fluid in object
        :param isVertical: tracker if object is drawn vertically
        :return:
        """

        # Initialize base classes
        super().__init__(parent=widget_parent, position=position, fluid=fluid, width= 40*1.75, height = 18*1.75, is_vertical=isVertical, is_being_edited = False)

        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widget_parent

        # State tracks whether the solenoid is open or closed
        self.state = 0

        # Label Position designates the side of the object the label will be placed
        # TODO: Make dictionary to designate, top, bottom, left and right positions
        self.labelPosition = 0

        # This is a fucking mess but I am too hella lazy to fix it rn
        # TODO: Make this not a mess
        if self.labelPosition == 0:
            self.label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
            if self.is_vertical == 0:
                self.label.move(self.position[0], self.position[1] - self.label.height())
            else:
                self.label.move(self.position[0] - self.label.width() / 2 + self.height / 2, self.position[1] - self.label.height())
        elif self.labelPosition == 1:
            self.label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                self.label.move(self.position[0] + self.width, self.position[1] - self.label.height() / 2 + self.height / 2)
            else:
                self.label.move(self.position[0] + self.height, self.position[1] - self.label.height() / 2 + self.width / 2)
        elif self.labelPosition == 2:
            self.label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
            if self.is_vertical == 0:
                self.label.move(self.position[0], self.position[1] + self.height)
            else:
                self.label.move(self.position[0] - self.label.width() / 2 + self.height / 2, self.position[1] + self.width)
        elif self.labelPosition == 3:
            self.label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            if self.is_vertical == 0:
                self.label.move(self.position[0] - self.width, self.position[1] - self.label.height() / 2 + self.height / 2)
            else:
                self.label.move(self.position[0] - self.width, self.position[1] - self.label.height() / 2 + self.width / 2)

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
            self.widget_parent.painter.setBrush(Constants.fluidColor[self.fluid])  # This function colors in a path
        else:
            self.widget_parent.painter.setBrush(0)

        # Sets line color
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])

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

        self.widget_parent.painter.drawPath(path)

        # This is debug, draws a box around the origin of object
        self.widget_parent.painter.fillRect(QRectF(self.position[0], self.position[1], 7, 7),
                                      Constants.fluidColor[self.fluid])

    def onClick(self):
        """
        When a solenoid is clicked this function is called
        """

        if self.widget_parent.window.is_editing == False:
            #Toggle state of solenoid
            self.toggle()
            #Tells widget painter to update screen
        else:
            if self.is_being_edited:
                self.widget_parent.controlsPanel.removeEditingObjects(self)
            else:
                self.widget_parent.controlsPanel.addEditingObjects(self)

        self.widget_parent.update()

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

    def updateLongName(self, name):
        """
        Updates long name and label of object
        """
        self.long_name = name
        self.label.setText(name)

    def updateLabelPosition(self, position):
        self.labelPosition = position

        label = self.label
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


    def plot_menu(self, event, button, menu):
        """
        Handler for context menu. These menus hand-off data plotting to plot windows
        :param event: default event from pyqt
        :param button: button instance this plot_menu is connected to
        :param menu: input QMenu object to display options on
        :return:
        """
        self.plotMenuActions = []
        for plot in self.widget_parent.parent.parent.plotWindow.plotList:
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