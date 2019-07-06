from PyQt5.QtCore import *

"""
Class to hold any constant value that is important to the entire program
"""

class Constants:

    # Dict of fluids. Example call: fluid["HE"] -> Returns 0
    fluid = {
        "HE" :  0,
        "Fuel": 1,
        "OX":   2,
        "LN2":  3
    }

    # Dict of fluid colors. Number on left should match value of fluid dict above. Example call: fluidColor[0] -> Returns Qt.white
    # TODO: check if dictionaries can be overloaded so number and string are accepted
    fluidColor = {
        0: Qt.white,
        1: Qt.magenta,
        2: Qt.cyan,
        3: Qt.green
    }