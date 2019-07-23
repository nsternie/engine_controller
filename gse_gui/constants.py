from PyQt5.QtCore import *
from PyQt5.QtGui import QColor

"""
Class to hold any constant value that is important to the entire program
"""

class Constants:

    # Dict of fluids. Example call: fluid["HE"] -> Returns 0
    # TODO: I'm picky so change to LOX
    fluid = {
        "HE" :  0,
        0: "HE",
        "Fuel": 1,
        1: "Fuel",
        "OX":   2,
        2: "OX",
        "LN2":  3,
        3: "LN2"
    }

    #List of fluids
    fluids = ["HE", "Fuel", "OX", "LN2"]

    # Dict of fluid colors. Number on left should match value of fluid dict above. Example call: fluidColor[0] -> Returns Qt.white
    fluidColor = {
        "HE": Qt.white,
        0: Qt.white,
        "Fuel": Qt.magenta,
        1: Qt.magenta,
        "OX": QColor(60,126,232),
        2: QColor(60,126,232),
        "LN2": Qt.green,
        3: Qt.green
    }