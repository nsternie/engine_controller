import sys
import csv
import time
import threading

from PyQt5.QtWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import numpy as np

class Plot(QWidget):
    """
    Widget to generate a plot window of data vs. time

    Creates a button for the plot in the parent window
    Plot is opened in new window
    """

    def __init__(self, name, parent=None):
        """
        Init for Plot class
        :param name: name of widget
        :param parent: parent of window
        """
        super().__init__(parent)
        self.parent = parent
        self.dataTuples = [] #Actual data storage
        self.dataTypes = [] #DataTypes on plot
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

    def link_data(self, button):
        """
        Link data from a specific button to a specific plot object
        If 2 datatypes are already on the plot, this doesn't work.
        :param button: button object to plot data from
        :return:
        """

        if len(self.dataTypes) == 2 and button.dataType not in self.dataTypes:
            self.message('This plot is full. Please select another plot')
            return

        if button.dataType not in self.dataTypes:
            self.dataTypes.append(button.dataType)
        self.dataTuples.append((button.dataFile,button.dataType, button.name))

    def read_data(self):
        """
        Read data at update_rate
        Update plot as new data is read
        Target function of thread
        :return:
        """
        while True:
            if self.openBool is True and len(self.dataTuples) > 0:
                self.message('checking for data')
                try:
                    dataArray = []
                    dataTypes = []
                    dataNames = []
                    for indx, dataTuple in enumerate(self.dataTuples):
                        dataFile = dataTuple[0]
                        dataType = dataTuple[1]
                        dataName = dataTuple[2]

                        data = []
                        with open(dataFile, 'r') as f:
                            rd = csv.reader(f)
                            for row in rd:
                                data.append([float(f) for f in row])

                        dataArray.append(data)
                        dataTypes.append(dataType)
                        dataNames.append(dataName)

                    self.plot_data(dataArray, dataTypes, dataNames)
                except FileNotFoundError:
                    print('Could not find {}'.format(dataFile))
                    pass
                except Exception as e:
                    e_type, e_obj, e_tb = sys.exc_info()
                    print('Got {} on line {}: {}'.format(e_type, e_tb.tb_lineno, e))
                    pass
            time.sleep(self.parent.update_rate)

    def plot_data(self, dataArray, dataTypes, dataNames):
        """
        Plot data on the figure
        :param data: data to plot [[[x,y],[x2,y2]...]]
        :param dataTypes: types of data to plot (array)
        :param dataNames: names of the line (array)
        :return:
        """
        self.message('plotting')

        ## Clear old data
        for dataType in dataTypes:
            indx = self.dataTypes.index(dataType)
            self.axes[indx].clear()

        for i in range(0,len(dataArray)):

            data = np.array(dataArray[i])
            dataType = dataTypes[i]
            dataName = dataNames[i]

            if dataType == 'Force':
                ylabel = "Force (N)"
                frm = 'b'
            elif dataType == 'Pressure':
                ylabel = "Pressure (psi)"
                frm = 'g'
            elif dataType == "Temperature":
                ylabel = 'Temperature (C)'
                frm = 'r'
            else:
                ylabel = 'State'
                frm = 'k'

            indx = self.dataTypes.index(dataType)

            self.axes[indx].set(ylabel=ylabel)
            self.axes[indx].plot(data[:, 0], data[:, 1], frm, label=dataName)

        self.fig.legend(loc='best')
        self.canvas.draw()

    def get_figure_dimensions(self):
        ## Determine plot size via window size
        parentWidth = self.parent.geometry().width()+25
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
        self.ax0 = self.fig.add_subplot(111)
        self.ax1 = self.ax0.twinx()
        self.axes = [self.ax0, self.ax1]

        ## Title
        self.fig.suptitle(self.name)