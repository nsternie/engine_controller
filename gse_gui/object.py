from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants
from plot import PlotButton

"""
Base class for GUI objects. Used to define parameters all GUI objects need
"""

class BaseObject(QPushButton):

    def __init__(self, parent: QWidget, position: QPointF, fluid: int, width: float, height : float, avionics_number: int = -1,
                 short_name: str = 'OX-SN-G07', safety_status: int = -1, long_name: str = 'LOX Dewar Drain',
                 is_vertical: bool = False, is_being_edited: bool = False):
        """
        Initializer for base class

        :param parent: parent widget
        :param position: position of icon on screen
        :param fluid: fluid in object
        :param width: width of object
        :param height: height of object
        :param avionics_number: avionics identifier
        :param short_name: abbreviated name on schematics
        :param safety_status: safety criticality
        :param long_name: human-readable name for display on screen
        :param is_vertical: tracker if object is drawn vertically
        :param is_being_edited: tracker if object is drawn vertically
        """
        super().__init__()

        self.widget_parent = parent # Important for drawing icon
        self._id = len(self.widget_parent.object_list) # Very important! DO NOT CHANGE FROM WHAT PROGRAM SET
        self.position = position
        self.fluid = fluid
        self.width = width
        self.height = height
        self.avionics_number = avionics_number
        self.short_name = short_name
        self.safety_status = safety_status
        self.long_name = long_name
        self.is_vertical = is_vertical
        self.is_being_edited = is_being_edited
        self.context_menu = QMenu(self.widget_parent)
        self.button = PlotButton(self.short_name, 'data.csv', 'Pressure', self.widget_parent)
        self.label = QLabel(self.widget_parent)

        self._initButton()
        self._initLabel()

    def _initButton(self):
        """
        Basic function that handles all the setup for the PlotButton
        Should only be called from __init__
        """
        # Create Button and style it
        self.button.setStyleSheet("background-color:transparent;border:0;")
        self.button.setContextMenuPolicy(Qt.CustomContextMenu)
        self.button.setToolTip(self.short_name + "\nState: Closed")

        self.button.move(self.position.x(), self.position.y())

        # FIXME: Context menu always appears in the top left
        self.context_menu.move(self.position.x(), self.position.y())

        # Add quitAction
        self.context_menu.addAction("Test RMB")

        # If the object is vertical set the button size accordingly
        # TODO: Update self.height and width if the solenoid is vertical instead of doing this
        if self.is_vertical:
            self.button.resize(self.height, self.width)
        else:
            self.button.resize(self.width, self.height)

        # Connect plot options to button context menu
        self.button.clicked.connect(lambda: self.onClick())
        self.button.customContextMenuRequested.connect(
            lambda *args: self.plot_menu(*args, self.button, self.context_menu)
        )
        self.button.show()

    def _initLabel(self):
        """
        Basic function that handles all the setup for the object label
        Should only be called from __init__
        """
        # Get font and set it
        font = QFont()
        font.setStyleStrategy(QFont.PreferAntialias)
        font.setFamily("Arial")
        font.setPointSize(23)
        self.label.setFont(font)

        # Sets the sizing of the label
        self.label.setFixedWidth(self.width)
        self.label.setFixedHeight(80)  # 80 Corresponds to three rows at this font type and size (Arial 23)
        self.label.setText(self.long_name)  # Solenoid long name
        self.label.setStyleSheet('color: white')
        self.label.setWordWrap(1)

        # Move the label into position
        self.label.move(self.position.x(), self.position.y())

        self.label.show()

    def setToolTip_(self, text):
        """
        Sets the toolTip of the button
        :param text: text to be set on the tooltip
        """
        self.button.setToolTip(self.short_name + "\n" + text)
        print(text)

    def setLongName(self, name):
        """
        Sets long name and label of object
        :param name: long_name of the object
        """
        self.long_name = name
        self.label.setText(name)

    def setShortName(self, name):
        """
        Sets short name of the object
        :param name: short_name of the object
        """
        self.short_name = name

    def onClick(self):
        """
        When a object is clicked this function is called
        This is base functionality, more functionality
        can be added by overriding this function in the
        child class
        """

        if self.widget_parent.window.is_editing:
            if self.is_being_edited:
                self.widget_parent.controlsPanel.removeEditingObjects(self)
            else:
                self.widget_parent.controlsPanel.addEditingObjects(self)

        # Tells widget painter to update screen
        self.widget_parent.update()

    def draw(self):
        """
        Draws the object
        Will almost always be overridden, this exists to
        show user it needs an override
        """

        # Draws simple 10 x 10 box
        self.widget_parent.painter.setPen(Constants.fluidColor[self.fluid])
        self.widget_parent.painter.fillRect(QRectF(self.position.x(), self.position.y(), 10, 10),
                                            Constants.fluidColor[self.fluid])
        
    def move(self, x_pos, y_pos):
        """
        Move solenoid to a new position

        :param x_pos: new x position
        :param y_pos: new y position
        """
        self.button.move(x_pos, y_pos)
        self.contextMenu.move(x_pos, y_pos)
        self.position = QPointF(x_pos, y_pos)

    def plot_menu(self, event, button, menu):
        """
        Handler for context menu. These menus hand-off data plotting to plot windows
        :param event: default event from pyqt
        :param button: button instance this plot_menu is connected to
        :param menu: input QMenu object to display options on
        :return:
        """
        self.plotMenuActions = []
        for plot in self.widget_parent.gui.plotWindow.plotList:
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


