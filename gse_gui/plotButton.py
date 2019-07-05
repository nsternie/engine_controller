from PyQt5.QtWidgets import *



class PlotButton(QPushButton):
    """
    Button class that holds a data file
    """

    ## Allowed data types, including solenoid state (0 or 1)
    allowed_data_types = ['Force','Temperature','Pressure','State']
    def __init__(self, name, dataFile, dataType, parent=None):
        """
        Init for PlotButton
        :param name: name on button
        :param dataFile: data file tied to button
        :param dataType: type of data in [Force, Temperature, Pressure, State]
        :param parent: parent window
        """
        super().__init__(name,parent)
        self.parent = parent
        self.name = name
        self.dataFile = dataFile
        self.dataType = dataType

        assert dataType in self.allowed_data_types
