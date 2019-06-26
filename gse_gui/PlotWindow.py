from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


from Plot import Plot

class PlotWindow(QMainWindow):
    """
    Window for plots
    """

    resized = pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(parent)
        ##TODO: Change this back to 5 Hz for operational use
        self.update_rate = 1/1 #Update plots at 1 Hz

        # Set geometry
        self.title = 'Plot View'
        self.left = 300
        self.top = 100
        self.width = 1100
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

        self.resized.connect(self.layout_widgets)

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
