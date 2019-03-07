import sys
import csv
import time
import threading

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np


class GUI(QMainWindow):
    """
    Parent class to hold all child widgets
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # Set geometry
        self.title = 'HOTFIRE GUI'
        self.left = 300
        self.top = 100
        self.width = 600
        self.height = 800

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Create grid and central widget
        grid = QGridLayout()
        cent = QWidget(self)
        self.setCentralWidget(cent)
        self.solenoid = SolenoidButton(self)
        self.plot = Plot(self)

        ## Place sub-widgets in a grid on the central widget
        grid.addWidget(self.solenoid, 0, 0)
        grid.addWidget(self.plot, 0, 1)
        cent.setLayout(grid)
        self.show()


class Window(QWidget):
    """
    Generic pop-up window class
    """

    def __init__(self, parent=None, openBool=False):
        """
        Initialize a window class
        :param parent: parent widget
        :param openBool: variable to keep track if window is open
        """
        super().__init__(parent)
        self.openBool = openBool

    def closeEvent(self, event):
        """
        Set openBool to false, then close the window
        :return:
        """
        self.openBool = False
        event.accept()
        print('closing window')


class Plot(QWidget):
    """
    Widget to generate a plot window of data vs. time

    Creates a button for the plot in the parent window
    Plot is opened in new window
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.data_file = 'data.csv'  ## Update this

        ## Generate plot button and link to pop-up window
        self.pushPlot = QPushButton(self)
        self.pushPlot.setText("Plot")
        self.pushPlot.clicked.connect(self.on_plot_click)

        self.plotWindow = Window(openBool=False)

        ## Open thread to update plot
        plotUpdater = threading.Thread(target=self.read_data,
                                       name='plotUpdater')
        plotUpdater.setDaemon(True)
        plotUpdater.start()

    def on_plot_click(self):
        """
        On click, generate plot and open window
        :return:
        """
        if self.plotWindow.openBool is False:
            self.create_main_frame()
            self.plotWindow.openBool = True

    def read_data(self):
        """
        Read data at 5Hz
        Update plot as new data is read
        Target function of thread
        :return:
        """
        while True:
            if self.plotWindow.openBool is True:
                print('checking for data')
                try:
                    data = []
                    with open(self.data_file, 'r') as f:
                        rd = csv.reader(f)
                        for row in rd:
                            data.append([float(f) for f in row])

                    self.plot_data(data)
                except FileNotFoundError:
                    print('Could not find {}'.format(self.data_file))
                    pass
                except Exception as e:
                    e_type, e_obj, e_tb = sys.exc_info()
                    print('Got {} on line {}: {}'.format(e_type, e_tb.tb_lineno, e))
                    pass
            time.sleep(0.2)

    def plot_data(self, data):
        """
        Plot data on the figure
        :param data: data to plot [[x,y],[x2,y2]...]
        :return:
        """
        print('plotting')
        data = np.array(data)

        self.axes.clear()
        self.axes.plot(data[:, 0], data[:, 1])
        self.canvas.draw()

    def create_main_frame(self):
        """
        Generate the plot figure
        :return:
        """
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)  # 5x4 inches, 100 dpi
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.plotWindow)
        self.plotWindow.show()

        ## Add axes
        self.axes = self.fig.add_subplot(111)


class SolenoidButton(QWidget):
    solenoidList = [[0, 'OX-SN-G01', 0, 0, [100, 100]], [1, 'OX-SN-G02', 0, 0, [100, 300]]]

    # Index 0: Button Refrence (TBA later in program)
    # Index 1: Avionics Number
    # Index 2: Long Name
    # Index 3: State (0 = closed, 1 = open)
    # Index 4: Saftey Status (0 = None, 1 = Warning, 2 = Critical)
    # Index 5: Position(LIST) [Xpos, Ypos]  ***WILL BE AUTO GENERATED FROM MATLAB****

    def __init__(self, parent=None):
        super().__init__(parent)

        self.parent = parent
        self.initObjects()

    def initObjects(self):
        # self.show()
        self.createSolenoidButtons(self.solenoidList)

    def createSolenoidButtons(self, list):
        # Height and Width of buttons
        height = 35
        width = 90

        # This is not the easiest way to iterate through the list but
        # is important for getting the sender when thenbutton is clicked
        for i in range(len(list)):
            print(i)
            # Create button and add it to first index in its list.
            button = QPushButton(self)
            list[i].insert(0, button)
            # Do all the graphics stuff
            button.move(list[i][5][0], list[i][5][1])
            # button.setStyleSheet("background-color:transparent;border:0;")
            button.setStyleSheet("background-color:red;")
            button.setToolTip(list[i][2])
            button.setAccessibleName(str(
                i))  # Sets the AccessibleName to the buttons corresponding index in solenoidList. Used for the slot.
            button.resize(width, height)
            button.clicked.connect(self.on_click)
            button.show()

    @pyqtSlot()
    def on_click(self):
        # Gets the senders(button) solenoidList index from the accessibleName
        index = int(self.sender().accessibleName())
        self.toggleSolenoid(index)

    def toggleSolenoid(self, index):

        if self.solenoidList[index][3] == 0:
            self.solenoidList[index][0].setStyleSheet("background-color:green;")
            self.solenoidList[index][3] = 1
        else:
            self.solenoidList[index][0].setStyleSheet("background-color:red;")
            self.solenoidList[index][3] = 0


if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = GUI()
    sys.exit(app.exec_())
