import sys
import csv
import time
import threading

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np

class GUI():
    """
        Parent class to hold all child windows
    """

    def __init__(self, parent=None):

        self.solenoidWindow = SolenoidWindow(self)
        self.plotWindow = PlotWindow()

class PlotWindow(QMainWindow):
    """
    Window for plots
    """

    resized = pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(parent)
        self.update_rate = 1/1 #Update plots at 1 Hz

        # Set geometry
        self.title = 'Plot View'
        self.left = 300
        self.top = 100
        self.width = 800
        self.height = 800

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Create grid and central widget
        self.grid = QGridLayout()
        self.cent = QWidget(self)
        self.setCentralWidget(self.cent)

        self.plotNumber = 4
        self.gen_plots(self.plotNumber)

        ## Place sub-widgets in a grid on the central widget
        self.cent.setLayout(self.grid)
        self.show()

        #self.resized.connect(self.layout_widgets)

    def gen_plots(self, num):
        """
        Add num plots to grid
        :param num: number of plots to generate
        :return:
        """
        self.plotList = []
        for i in range(0,int(self.plotNumber/2)):
            for j in range(0,int(self.plotNumber/2)):
                plot = Plot('plot{}-{}'.format(i,j),parent=self)
                self.plotList.append(plot)
                self.grid.addWidget(plot, i, j)

    def layout_widgets(self):
        """
        Update grid sizing and plot sizing for resized window
        :return:
        """

        self.grid = QGridLayout()

        ## Update plot sizes
        i = 0
        j = 0
        for plot in self.plotList:
            self.grid.addWidget(plot, i, j)
            plot.update_figure()

            ## Keep counters between 0 and 1
            i+=1
            j+=1

            if i > 1:
                i = 0
            if j > 1:
                j = 0

        self.cent.setLayout(self.grid)

    def closeEvent(self, event):
        """
        Set openBool to false, then close the window
        :return:
        """
        for plot in self.plotList:
            plot.openBool = False
        event.accept()
        print('closing window')

    def resizeEvent(self, event):
        self.resized.emit()
        super().resizeEvent(event)

class SolenoidWindow(QMainWindow):
    """
    Window for solenoid control
    """

    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        # Set geometry
        self.title = 'Solenoid Control'
        self.left = 300
        self.top = 100
        self.width = 600
        self.height = 800

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.solenoid = SolenoidWidget(self)
        self.show()

class Plot(QWidget):
    """
    Widget to generate a plot window of data vs. time

    Creates a button for the plot in the parent window
    Plot is opened in new window
    """

    def __init__(self, name, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.dataFile = None
        self.openBool = True
        self.name = name

        self.create_main_frame()

        ## Open thread to update plot
        plotUpdater = threading.Thread(target=self.read_data,
                                       name='plotUpdater')
        plotUpdater.setDaemon(True)
        plotUpdater.start()

    def message(self, msg):
        """
        Output a message to console with the name of this instance
        :param msg: string to output
        :return:
        """
        print('{}: {}'.format(self.name,msg))

    def read_data(self):
        """
        Read data at update_rate
        Update plot as new data is read
        Target function of thread
        :return:
        """
        while True:
            if self.openBool is True and self.dataFile is not None:
                self.message('checking for data')
                try:
                    data = []
                    with open(self.dataFile, 'r') as f:
                        rd = csv.reader(f)
                        for row in rd:
                            data.append([float(f) for f in row])

                    self.plot_data(data)
                except FileNotFoundError:
                    print('Could not find {}'.format(self.dataFile))
                    pass
                except Exception as e:
                    e_type, e_obj, e_tb = sys.exc_info()
                    print('Got {} on line {}: {}'.format(e_type, e_tb.tb_lineno, e))
                    pass
            time.sleep(self.parent.update_rate)

    def plot_data(self, data):
        """
        Plot data on the figure
        :param data: data to plot [[x,y],[x2,y2]...]
        :return:
        """
        self.message('plotting')
        data = np.array(data)

        self.axes.clear()
        self.axes.plot(data[:, 0], data[:, 1])
        self.canvas.draw()

    def get_figure_dimensions(self):
        ## Determine plot size via window size
        parentWidth = self.parent.geometry().width()
        parentHeight = self.parent.geometry().height()-10
        monitorDPIX = self.physicalDpiX()
        monitorDPIY = self.physicalDpiY()

        figWidth = float(parentWidth) / (self.parent.plotNumber/2) / monitorDPIX
        figHeight = float(parentHeight) / (self.parent.plotNumber/2) / monitorDPIY

        return figWidth, figHeight

    def update_figure(self):
        figWidth, figHeight = self.get_figure_dimensions()

        self.fig.set_size_inches(figWidth, figHeight, forward=True)

    def create_main_frame(self):
        """
        Generate the plot figure
        :return:
        """
        self.dpi = 100

        ## Determine plot size via window size
        figWidth, figHeight = self.get_figure_dimensions()

        self.fig = Figure((figWidth, figHeight), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)

        ## Add axes
        self.axes = self.fig.add_subplot(111)

        ## Title
        self.fig.suptitle(self.name)

class PlotButton(QPushButton):
    """
    Button class that holds a data file
    """
    def __init__(self, name, dataFile, parent=None):
        super().__init__(name,parent)
        self.parent = parent
        self.dataFile = dataFile


class SolenoidWidget(QWidget):
    solXOffsetList = []
    solYOffsetList = []
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

    # ****THESE SHOULD BE REORDERED******

    # Index 0: Button Refrence (TBA later in program)
    # Index 1: Avionics Number
    # Index 2: Short Name
    # Index 3: State (0 = closed, 1 = open)
    # Index 4: Saftey Status (0 = None, 1 = Warning, 2 = Critical)
    # Index 5: Position(LIST) [Xpos, Ypos]  ***WILL BE AUTO GENERATED FROM MATLAB****
    # Index 6: Long Name
    # Index 7: Solenoid & Label Positioning Data(LIST) [Solenoid Orientation, Label Relative Pos] (Sol Orientation: 0 = Horizontal, 1 = Vertical
    # Label Relative Pos: 0 = Above Solenoid, 1 = Right, 2 = Below, 3 = Left) ***Yes I know this is a stupid way of doing this***
    # Index 8: Label Refrence

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent


        self.title = 'HOTFIRE GUI'
        self.left = 300
        self.top = 100
        self.width = 1680
        self.height = 1050
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

        self.initObjects()

    def initObjects(self):
        self.loadCsv('Sol.csv')
        self.updateSolOffsets()
        self.createSolenoidButtons(self.solenoidList)

        ## Create test ducer button
        self.ducerButton = PlotButton("Test",'data.csv',self)
        self.ducerButton.setContextMenuPolicy(Qt.CustomContextMenu)
        self.ducerButton.customContextMenuRequested.connect(
            lambda *args, button=self.ducerButton: self.plot_menu(*args, button)
        )
        self.ducerButton.show()

    def plot_menu(self, event, button):
        """
        Handler for context menu. These menus hand-off data plotting to plot windows
        :param event: default event from pyqt
        :param button: button instance this plot_menu is connected to
        :return:
        """
        self.plotMenu = QMenu(self)
        self.plotMenuActions = []
        for plot in self.parent.parent.plotWindow.plotList:
            action = QAction(plot.name)
            self.plotMenuActions.append(action)

            link_plot_wrapper = lambda : self.link_plot(plot)
            self.plotMenuActions[-1].triggered.connect(
                lambda *args, p=plot: self.link_plot(p, button)
            )

            self.plotMenu.addAction(self.plotMenuActions[-1])

        self.plotMenu.exec_(self.mapToGlobal(event))

    def link_plot(self, plot, button):
        """
        Link a Plot object to a given data file
        :param plot: plot object that needs a link to a data file
        :param button: button instance that was clicked on
        :return:
        """
        plot.dataFile = button.dataFile

    def updateSolOffsets(self):
        for i in range(len(self.solenoidList)):
            print(int(self.solXOffsetList[i]), int(self.solYOffsetList[i]))
            # Due to shit coding the index 5 is actually 4 here
            self.solenoidList[i][4][0] = int(self.solXOffsetList[i])
            self.solenoidList[i][4][1] = int(self.solYOffsetList[i])

    # Handles all the drawing of objects. Auto Updates
    qp = QPainter()

    def paintEvent(self, e):

        self.qp.begin(self)
        self.qp.setRenderHint(QPainter.HighQualityAntialiasing)

        self.drawSolenoid(self.qp)

        self.qp.end()

    def drawSolenoid(self, qp):
        # Solenoid can be drawn with just lines or filled in.
        # Filled in represents it open, just an outline is closed.

        # These values assume if the solenoid is drawn horizontally
        height = 30;
        width = 75;

        for i in range(len(self.solenoidList)):
            path = QPainterPath()

            xOffset = self.solenoidList[i][5][0]
            yOffset = self.solenoidList[i][5][1]

            if self.solenoidList[i][3] == 1:
                qp.setBrush(Qt.cyan)
            else:
                qp.setBrush(0)

            qp.setPen(Qt.cyan)
            path.moveTo(xOffset, yOffset)

            # = 0 -> Draw horizontally
            if self.solenoidList[i][7][0] == 0:
                path.lineTo(xOffset, yOffset + height)
                path.lineTo(xOffset + width, yOffset)
                path.lineTo(xOffset + width, yOffset + height)
                path.lineTo(xOffset, yOffset)
            else:
                path.lineTo(xOffset + height, yOffset)
                path.lineTo(xOffset, yOffset + width)
                path.lineTo(xOffset + height, yOffset + width)
                path.lineTo(xOffset, yOffset)

            qp.drawPath(path)

    # Creates Solenoid Button and Corresponding Label
    def createSolenoidButtons(self, list):
        # Height and Width of buttons
        height = 30;
        width = 75;

        # This is not the easiest way to iterate through the list but
        # is important for getting the sender when thenbutton is clicked
        for i in range(len(list)):
            # Create button and add it to first index in its list.
            button = QPushButton(self)
            list[i].insert(0, button)
            # Do all the graphics stuff
            button.move(list[i][5][0], list[i][5][1])
            button.setStyleSheet("background-color:transparent;border:0;")
            # button.setStyleSheet("background-color:red;")
            button.setToolTip(list[i][2] + "\nState: Closed")
            button.setAccessibleName(str(
                i))  # Sets the AccessibleName to the buttons corresponding index in solenoidList. Used for the slot.
            if list[i][7][0] == 0:
                button.resize(width, height)
            else:
                button.resize(height, width)
            button.clicked.connect(self.on_click)
            button.setContextMenuPolicy(Qt.DefaultContextMenu)
            button.show()

            label = QLabel(self)
            list[i].insert(8, label)

            font = QFont()
            font.setStyleStrategy(QFont.PreferAntialias)
            font.setFamily("Arial")
            font.setPointSize(23)
            label.setFont(font)

            label.setFixedWidth(width)
            label.setFixedHeight(80)  # 80 Coressesponds to three rows at this font type and size (Arial 23)
            label.setText(list[i][6])
            label.setWordWrap(1)
            # label.setStyleSheet("background-color:red;")

            if list[i][7][1] == 0:
                label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
                if list[i][7][0] == 0:
                    label.move(list[i][5][0], list[i][5][1] - label.height())
                else:
                    label.move(list[i][5][0] - label.width() / 2 + height / 2, list[i][5][1] - label.height())
            elif list[i][7][1] == 1:
                label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
                if list[i][7][0] == 0:
                    label.move(list[i][5][0] + width, list[i][5][1] - label.height() / 2 + height / 2)
                else:
                    label.move(list[i][5][0] + height, list[i][5][1] - label.height() / 2 + width / 2)
            elif list[i][7][1] == 2:
                label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
                if list[i][7][0] == 0:
                    label.move(list[i][5][0], list[i][5][1] + height)
                else:
                    label.move(list[i][5][0] - label.width() / 2 + height / 2, list[i][5][1] + width)
            elif list[i][7][1] == 3:
                label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
                if list[i][7][0] == 0:
                    label.move(list[i][5][0] - width, list[i][5][1] - label.height() / 2 + height / 2)
                else:
                    label.move(list[i][5][0] - width, list[i][5][1] - label.height() / 2 + width / 2)

            label.show()

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        quitAction = menu.addAction("Test RMB")
        action = menu.exec_(self.mapToGlobal(event.pos()))

    @pyqtSlot()
    def on_click(self):
        # Gets the senders(button) solenoidList index from the accessibleName
        index = int(self.sender().accessibleName())
        print(index)
        self.toggleSolenoid(index)
        self.update()

    # Called when a solenoid is clicked. Toggles on/ off
    def toggleSolenoid(self, index):
        if self.solenoidList[index][3] == 0:
            self.solenoidList[index][3] = 1
            self.solenoidList[index][0].setToolTip(self.solenoidList[index][2] + "\nState: Open")
        else:
            self.solenoidList[index][3] = 0
            self.solenoidList[index][0].setToolTip(self.solenoidList[index][2] + "\nState: Closed")

    def loadCsv(self, filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            your_list = list(reader)
        self.solXOffsetList = your_list[0]
        self.solYOffsetList = your_list[1]


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
