from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from PlotButton import PlotButton


class SolenoidHelper:
    solXPosList = []
    solYPosList = []
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

    # Index 0: Button Reference (TBA later in program)
    # Index 1: Avionics Number
    # Index 2: Short Name
    # Index 3: State (0 = closed, 1 = open)
    # Index 4: Safety Status (0 = None, 1 = Warning, 2 = Critical)
    # Index 5: Position(LIST) [Xpos, Ypos]  ***WILL BE AUTO GENERATED FROM MATLAB****
    # Index 6: Long Name
    # Index 7: Solenoid & Label Positioning Data(LIST) [Solenoid Orientation, Label Relative Pos] (Sol Orientation: 0 = Horizontal, 1 = Vertical
    # Label Relative Pos: 0 = Above Solenoid, 1 = Right, 2 = Below, 3 = Left) ***Yes I know this is a stupid way of doing this***
    # Index 8: Label Reference


    def createObjects(self, controlsWidget, csvObjectData):

        self.updateSolenoidPositions(csvObjectData)
        self.createSolenoidButtons(controlsWidget)
        self.createSolenoidLables(controlsWidget)

    def createSolenoidButtons(self, controlsWidget):
        # Height and Width of buttons
        # Will be eventually grabbed by matlab csv
        height = 30;
        width = 75;

        for i in range(len(self.solenoidList)):
            # Creates a button with the control widget as the parent
            # This is important for getting the buttons sender when
            # it is clicked.
            tempButton = QPushButton(controlsWidget)

            # Adds the button to the solenoid list
            self.solenoidList[i].insert(0, tempButton)

            # Move button to position set by matlab csv
            tempButton.move(self.solenoidList[i][5][0], self.solenoidList[i][5][1])

            # Set button size
            # Will be made automatic by matlab csv data
            # Checks orrientation of Solenoid
            if self.solenoidList[i][7][0] == 0:
                tempButton.resize(width,height)
            else:
                tempButton.resize(height,width)

            # Graphics Things
            tempButton.setStyleSheet("background-color:transparent;border:0;")
            tempButton.setToolTip(self.solenoidList[i][2] + "\nState: Closed")
            tempButton.setContextMenuPolicy(Qt.DefaultContextMenu)

            # Sets the accessible name of the button to its index in solenoid list. This is to check what button was
            # clicked
            tempButton.setAccessibleName(str(i))

            # Sets function to call when button is clicked. main window handles clicks
            tempButton.clicked.connect(controlsWidget.on_click)

            # Shows the button
            tempButton.show()

    def createSolenoidLables(self, controlsWidget):
        # Height and Width of buttons
        # Will be eventually grabbed by matlab csv
        height = 30;
        width = 75;

        for i in range(len(self.solenoidList)):

            # Creates Label
            tempLabel = QLabel(controlsWidget)

            # Inserts Label refrence into main list
            self.solenoidList[i].insert(8, tempLabel)

            # Sets the font for the label
            font = QFont()
            font.setStyleStrategy(QFont.PreferAntialias)
            font.setFamily("Arial")
            font.setPointSize(23)
            tempLabel.setFont(font)

            # Sets the sizing of the label
            tempLabel.setFixedWidth(width)
            tempLabel.setFixedHeight(80)  # 80 Corresponds to three rows at this font type and size (Arial 23)
            tempLabel.setText(self.solenoidList[i][6]) # Solenoid long name
            tempLabel.setWordWrap(1)

            # Wild if statement structure for positioning the labels next to vertical or horizontal solenoids
            # This seems messy but don't have a better way of doing it at the moment
            # To save my sanity:
            list = self.solenoidList
            if list[i][7][1] == 0:
                tempLabel.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
                if list[i][7][0] == 0:
                    tempLabel.move(list[i][5][0], list[i][5][1] - tempLabel.height())
                else:
                    tempLabel.move(list[i][5][0] - tempLabel.width() / 2 + height / 2, list[i][5][1] - tempLabel.height())
            elif list[i][7][1] == 1:
                tempLabel.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
                if list[i][7][0] == 0:
                    tempLabel.move(list[i][5][0] + width, list[i][5][1] - tempLabel.height() / 2 + height / 2)
                else:
                    tempLabel.move(list[i][5][0] + height, list[i][5][1] - tempLabel.height() / 2 + width / 2)
            elif list[i][7][1] == 2:
                tempLabel.setAlignment(Qt.AlignCenter | Qt.AlignTop)
                if list[i][7][0] == 0:
                    tempLabel.move(list[i][5][0], list[i][5][1] + height)
                else:
                    tempLabel.move(list[i][5][0] - tempLabel.width() / 2 + height / 2, list[i][5][1] + width)
            elif list[i][7][1] == 3:
                tempLabel.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
                if list[i][7][0] == 0:
                    tempLabel.move(list[i][5][0] - width, list[i][5][1] - tempLabel.height() / 2 + height / 2)
                else:
                    tempLabel.move(list[i][5][0] - width, list[i][5][1] - tempLabel.height() / 2 + width / 2)

            tempLabel.show()

    def updateSolenoidPositions(self, csvObjectData):
        self.solXPosList = csvObjectData[1][0]
        self.solYPosList = csvObjectData[1][1]

        for i in range(len(self.solenoidList)):
            # Due to shit coding the index 5 is actually 4 here
            #0 -> x pos, 1 -> y pos
            self.solenoidList[i][4][0] = int(self.solXPosList[i])
            self.solenoidList[i][4][1] = int(self.solYPosList[i])


    def drawSolenoids(self, qp, objScale):
        # Solenoid can be drawn with just lines or filled in.
        # Filled in represents it open, just an outline is closed.

        # These values assume if the solenoid is drawn horizontally
        # Will eventually be grabbed from matlab csv data
        height = 18 * objScale;
        width = 40 * objScale;

        for i in range(len(self.solenoidList)):
            # Define a new paint path
            path = QPainterPath()

            # To make coding easier
            xPos = self.solenoidList[i][5][0]
            yPos = self.solenoidList[i][5][1]

            # If solenoid is open color it in
            if self.solenoidList[i][3] == 1:
                qp.setBrush(Qt.cyan) #This function colors in a path
            else:
                qp.setBrush(0)

            # Need to update and grab real color from sol list
            qp.setPen(Qt.cyan)

            #Move path to starting position
            path.moveTo(xPos, yPos) #Top left corner

            # = 0 -> Draw horizontally
            if self.solenoidList[i][7][0] == 0:
                path.lineTo(xPos, yPos + height)        # Straight Down
                path.lineTo(xPos + width, yPos)         # Diag to upper right
                path.lineTo(xPos + width, yPos + height)# Straight Up
                path.lineTo(xPos, yPos)
            else: # Draw verticaly
                path.lineTo(xPos + height, yPos)
                path.lineTo(xPos, yPos + width)
                path.lineTo(xPos + height, yPos + width)
                path.lineTo(xPos, yPos)

            qp.drawPath(path)

    # Called when a solenoid is clicked. Toggles on/ off
    def toggleSolenoid(self, index):
        if self.solenoidList[index][3] == 0:
            self.solenoidList[index][3] = 1
            self.solenoidList[index][0].setToolTip(self.solenoidList[index][2] + "\nState: Open")
        else:
            self.solenoidList[index][3] = 0
            self.solenoidList[index][0].setToolTip(self.solenoidList[index][2] + "\nState: Closed")








    # OLD HERE FOR REFERENCE
    #
    #
    #
    # def initObjects(self, controlsWidget):
    #
    #
    #     self.solXOffsetList = self.csvData[1][0]
    #     self.solYOffsetList = self.csvData[1][1]
    #
    #     self.updateSolOffsets()
    #     self.createSolenoidButtons(self.solenoidList, controlsWidget)
    #
    #     ## Create test ducer button
    #     self.ducerButton1 = PlotButton("Press1", 'data.csv', 'Pressure', controlsWidget)
    #     self.ducerButton1.setContextMenuPolicy(Qt.CustomContextMenu)
    #     self.ducerButton1.customContextMenuRequested.connect(
    #         lambda *args, button=self.ducerButton1: self.plot_menu(*args, button)
    #     )
    #     self.ducerButton1.show()
    #
    #     self.ducerButton2 = PlotButton("Temp1", 'data1.csv', 'Temperature', controlsWidget)
    #     self.ducerButton2.setContextMenuPolicy(Qt.CustomContextMenu)
    #     self.ducerButton2.customContextMenuRequested.connect(
    #         lambda *args, button=self.ducerButton2: self.plot_menu(*args, controlsWidget)
    #     )
    #     self.ducerButton2.move(100,0)
    #     self.ducerButton2.show()
    #
    #     self.ducerButton3 = PlotButton("Press2", 'data2.csv', 'Pressure', controlsWidget)
    #     self.ducerButton3.setContextMenuPolicy(Qt.CustomContextMenu)
    #     self.ducerButton3.customContextMenuRequested.connect(
    #         lambda *args, button=self.ducerButton3: self.plot_menu(*args, button)
    #     )
    #     self.ducerButton3.move(200, 0)
    #     self.ducerButton3.show()
    #
    #     self.ducerButton4 = PlotButton("Force1", 'data3.csv', 'Force', controlsWidget)
    #     self.ducerButton4.setContextMenuPolicy(Qt.CustomContextMenu)
    #     self.ducerButton4.customContextMenuRequested.connect(
    #         lambda *args, button=self.ducerButton4: self.plot_menu(*args, button)
    #     )
    #     self.ducerButton4.move(300, 0)
    #     self.ducerButton4.show()
    #
    # def plot_menu(self, event, button):
    #     """
    #     Handler for context menu. These menus hand-off data plotting to plot windows
    #     :param event: default event from pyqt
    #     :param button: button instance this plot_menu is connected to
    #     :return:
    #     """
    #     self.plotMenu = QMenu(self)
    #     self.plotMenuActions = []
    #     for plot in self.parent.parent.plotWindow.plotList:
    #         action = QAction(plot.name)
    #         self.plotMenuActions.append(action)
    #
    #         link_plot_wrapper = lambda : self.link_plot(plot)
    #         self.plotMenuActions[-1].triggered.connect(
    #             lambda *args, p=plot: self.link_plot(p, button)
    #         )
    #
    #         self.plotMenu.addAction(self.plotMenuActions[-1])
    #
    #     self.plotMenu.exec_(self.mapToGlobal(event))
    #
    # def link_plot(self, plot, button):
    #     """
    #     Link a Plot object to a given data file
    #     :param plot: plot object that needs a link to a data file
    #     :param button: button instance that was clicked on
    #     :return:
    #     """
    #     plot.link_data(button)
    #
    # def updateSolOffsets(self):
    #     for i in range(len(self.solenoidList)):
    #         print(int(self.solXOffsetList[i]), int(self.solYOffsetList[i]))
    #         # Due to shit coding the index 5 is actually 4 here
    #         self.solenoidList[i][4][0] = int(self.solXOffsetList[i])
    #         self.solenoidList[i][4][1] = int(self.solYOffsetList[i])
    #
    #
    # Creates Solenoid Button and Corresponding Label
    # def createSolenoidButtons(self, list, controlsWidget):
    #     # Height and Width of buttons
    #     height = 30;
    #     width = 75;
    #
    #     # This is not the easiest way to iterate through the list but
    #     # is important for getting the sender when thenbutton is clicked
    #     for i in range(len(list)):
    #         # Create button and add it to first index in its list.
    #         button = QPushButton(controlsWidget)
    #         list[i].insert(0, button)
    #         # Do all the graphics stuff
    #         button.move(list[i][5][0], list[i][5][1])
    #         button.setStyleSheet("background-color:transparent;border:0;")
    #         # button.setStyleSheet("background-color:red;")
    #         button.setToolTip(list[i][2] + "\nState: Closed")
    #         button.setAccessibleName(str(i))  # Sets the AccessibleName to the buttons corresponding index in solenoidList. Used for the slot.
    #         if list[i][7][0] == 0:
    #             button.resize(width, height)
    #         else:
    #             button.resize(height, width)
    #         button.clicked.connect(controlsWidget.on_click)
    #         button.setContextMenuPolicy(Qt.DefaultContextMenu)
    #         button.show()
    #
    #         label = QLabel(controlsWidget)
    #         list[i].insert(8, label)
    #
    #         font = QFont()
    #         font.setStyleStrategy(QFont.PreferAntialias)
    #         font.setFamily("Arial")
    #         font.setPointSize(23)
    #         label.setFont(font)
    #
    #         label.setFixedWidth(width)
    #         label.setFixedHeight(80)  # 80 Coressesponds to three rows at this font type and size (Arial 23)
    #         label.setText(list[i][6])
    #         label.setWordWrap(1)
    #         # label.setStyleSheet("background-color:red;")
    #
    #         if list[i][7][1] == 0:
    #             label.setAlignment(Qt.AlignCenter | Qt.AlignBottom)
    #             if list[i][7][0] == 0:
    #                 label.move(list[i][5][0], list[i][5][1] - label.height())
    #             else:
    #                 label.move(list[i][5][0] - label.width() / 2 + height / 2, list[i][5][1] - label.height())
    #         elif list[i][7][1] == 1:
    #             label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
    #             if list[i][7][0] == 0:
    #                 label.move(list[i][5][0] + width, list[i][5][1] - label.height() / 2 + height / 2)
    #             else:
    #                 label.move(list[i][5][0] + height, list[i][5][1] - label.height() / 2 + width / 2)
    #         elif list[i][7][1] == 2:
    #             label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
    #             if list[i][7][0] == 0:
    #                 label.move(list[i][5][0], list[i][5][1] + height)
    #             else:
    #                 label.move(list[i][5][0] - label.width() / 2 + height / 2, list[i][5][1] + width)
    #         elif list[i][7][1] == 3:
    #             label.setAlignment(Qt.AlignCenter | Qt.AlignCenter)
    #             if list[i][7][0] == 0:
    #                 label.move(list[i][5][0] - width, list[i][5][1] - label.height() / 2 + height / 2)
    #             else:
    #                 label.move(list[i][5][0] - width, list[i][5][1] - label.height() / 2 + width / 2)
    #
    #         label.show()




