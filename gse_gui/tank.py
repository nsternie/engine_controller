from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants
from MathHelper import MathHelper
from object import BaseObject
from plot import PlotButton


"""
Class to handle all tank objects and their functionality 
"""

# TODO: Tanks need to be more similar to solenoids so the base object can be expanded
class Tank(QPushButton, BaseObject):

    def __init__(self, widgetParent, position, fluid):

        """
        Init the solenoid object
        :param widgetParent: widget this object will be added to
        :param position: position of icon on screen
        :param fluid: fluid in object
        :return:
        """
        ## Initialize underlying class
        super().__init__(parent=widgetParent, position=position, fluid=fluid, is_vertical=False, is_being_edited = False)

        self.widgetParent = widgetParent  # Important for drawing icon

        # This ID could be calculated better to avoid repeats but it works for now
        self._id = len(self.widgetParent.tank_list)  # Very important! DO NOT CHANGE FROM WHAT PROGRAM SET

        # Should be grabbed by csv and scaled
        # TODO: Grab height and width from csv file
        # TODO: Grab object scale from widgetParent
        self.height = 170 * 1.75
        self.width = 88 * 1.75

        # Tracks the percentage of fluid in the tank
        self.fillPercent = 0

        # Create Button and style it
        # TODO: Make the button include just the tank region and not everything else
        button = PlotButton(self.short_name, 'data.csv', 'Pressure', self.widgetParent)
        button.setStyleSheet("background-color:transparent;border:0;")
        button.setContextMenuPolicy(Qt.CustomContextMenu)
        button.setToolTip(self.short_name + "\nState: Closed")

        button.move(self.position[0], self.position[1] - 20)
        self.contextMenu = QMenu(self.widgetParent)
        # FIXME: Context menu always appears in the top left
        self.contextMenu.move(self.position[0], self.position[1])

        ## Add quitAction
        self.contextMenu.addAction("Test RMB")

        # If the sol is vertical set the button size accordingly
        # TODO: Update self.height and width if the solenoid is vertical instead of doing this
        if self.is_vertical:
            button.resize(self.height, self.width)
        else:
            button.resize(self.width, self.height)

        # Connect plot options to button context menu
        button.clicked.connect(lambda: self.onClick())
        button.customContextMenuRequested.connect(
            lambda *args: self.plot_menu(*args, button, self.contextMenu)
        )
        button.show()
        self.button = button


        # Holder for right now
        # TODO: Implement the rest of this
        self.labelPosition = 0

    def onClick(self):
        """
        When a tank is clicked this function is called
        This is really only useful for adding plots and
        selecting the tank for editing
        """

        if self.widgetParent.window.is_editing == False:
            # This is for testing and will normally be used with capacitive level sensor
            self.fillPercent += .05

        else:
            # TODO: Make this added to toggle function somehow
            if self.is_being_edited:
                self.widgetParent.controlsPanel.removeEditingObjects(self)
            else:
                self.widgetParent.controlsPanel.addEditingObjects(self)

        # Tells widget painter to update screen
        self.widgetParent.update()


    def updateLongName(self, name):
        """
        Updates long name and label of object
        """
        self.long_name = name
        # TODO: Add label for tank
        #self.label.setText(name)


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

    def draw(self):
        """
        Draws the solenoid icon on screen
        """

        # Height of curved arc at top and bottom of tank
        arcHeight = 20

        # Draws the tank outline
        path = QPainterPath()
        self.widgetParent.painter.setPen(Constants.fluidColor[self.fluid])

        path.moveTo(self.position[0],self.position[1])
        path.arcTo(QRectF(self.position[0], self.position[1] - arcHeight, self.width, arcHeight * 2), 180, -180)
        path.lineTo(self.position[0] + self.width, self.position[1]+ self.height - 2 * arcHeight)
        path.arcTo(QRectF(self.position[0] + self.width, self.position[1] + self.height - arcHeight - 2 * arcHeight, - self.width, arcHeight * 2), 180, 180)
        path.lineTo(self.position[0], self.position[1])

        self.widgetParent.painter.drawPath(path)

        self.widgetParent.painter.fillRect(QRectF(self.position[0], self.position[1], 7, 7), Constants.fluidColor[self.fluid])

        # End tank outline draw

        # Fill in bottom arc
        path = QPainterPath()
        self.widgetParent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widgetParent.painter.setBrush(Constants.fluidColor[self.fluid])

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

        self.widgetParent.painter.drawPath(path)
        # End fill in bottom arc

        # Fill in tank body
        # Maps fill percentage to the height of the body to fill
        bodyFillHeight = MathHelper.mapValue(self.fillPercent, arcHeight / self.height, 1 - arcHeight / self.height, 0,
                                       self.height - 2 * arcHeight)

        self.widgetParent.painter.fillRect(QRectF(self.position[0], self.position[1] - 2 * arcHeight + self.height - bodyFillHeight, self.width, bodyFillHeight), Constants.fluidColor[self.fluid])
        # End fill in tank body

        # Fill in top arc
        path = QPainterPath()
        self.widgetParent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widgetParent.painter.setBrush(Constants.fluidColor[self.fluid])

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

        self.widgetParent.painter.drawPath(path)

        self.widgetParent.painter.setBrush(0)

        # End fill in top arc
