from PyQt5.QtCore import *

"""
Class to hold any constant value that is important to the entire program
"""

class Constants:

    # Dict of fluids. Example call: fluid["HE"] -> Returns 0
    # TODO: I'm picky so change to LOX
    fluid = {
        "HE" :  0,
        "Fuel": 1,
        "OX":   2,
        "LN2":  3
    }

    # Dict of fluid colors. Number on left should match value of fluid dict above. Example call: fluidColor[0] -> Returns Qt.white
    fluidColor = {
        "HE": Qt.white,
        0: Qt.white,
        "Fuel": Qt.magenta,
        1: Qt.magenta,
        "OX": Qt.cyan,
        2: Qt.cyan,
        "LN2": Qt.green,
        3: Qt.green
    }