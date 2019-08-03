from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from overrides import overrides

"""
Provides custom functionality for labels
"""

class CustomLabel(QLabel):

    def __init__(self, widget_parent, object_, position_string: str = "Top", is_vertical: bool = False):
        super().__init__(widget_parent)

        self.widget_parent = widget_parent
        self.object_ = object_
        self.is_vertical = is_vertical
        self.position_string = position_string

    def setFixedSize_(self):
        """
        Sets the label size to be exactly the size of the text
        """
        if self.is_vertical:
            self.setFixedSize(
                self.fontMetrics().boundingRect(self.text()).height(),
                self.fontMetrics().boundingRect(self.text()).width())
        else:
            self.setFixedSize(
                self.fontMetrics().boundingRect(self.text()).width(),
                self.fontMetrics().boundingRect(self.text()).height())

    def setIsVertical(self, is_vertical):
        """
        Sets if the label is vertical or not
        :param is_vertical: is label vertical
        """
        self.is_vertical = is_vertical
        self.setFixedSize_()

    # TODO: Add assert for the string positions
    def moveToPosition(self, position_string: str = None):
        """
        Sets position string ang moves label to a position relative to its object.
        Can pass in option arg to set new value to position_string, otherwise will set
        position based on last set value
        :param position_string: string representing position of label. Can be "Top", "Bottom", "Right", "Left"
        """

        # Make sure the label width and height are up to date
        self.setFixedSize_()

        if not position_string is None:
            self.position_string = position_string

        if self.position_string == "Top":
            self.move(self.getXCenterPosition(), self.object_.position.y() + self.object_.height - self.height())
        elif self.position_string == "Bottom":
            self.move(self.getXCenterPosition(), self.object_.position.y() + self.object_.height + 3)
        elif self.position_string == "Right":
            self.move(self.object_.position.x() + self.object_.width, self.getYCenterPosition())
        elif self.position_string == "Left":
            self.move(self.object_.position.x() - self.width(), self.getYCenterPosition())


    def getXCenterPosition(self):
        """
        Gets what x position the label needs to be placed at to be centered on its base object
        """
        return self.object_.position.x() + (self.object_.width / 2) - (self.width() / 2)

    def getYCenterPosition(self):
        """
        Gets what y position the label needs to be placed at to be centered on its base object
        """
        return self.object_.position.y() + (self.object_.height / 2) - (self.height() / 2)


    @overrides
    def paintEvent(self, event):
        """
        Overrides the default painter to enable painting the text vertically

        :param event: event passed along
        """
        rotation = 90
        painter = QPainter(self)
        painter.setPen(Qt.white)
        painter.rotate(rotation * self.is_vertical)
        if self.text:
            if self.is_vertical:
                painter.drawText(QPoint(0,0), self.text())
            else:
                painter.drawText(QPoint(0, self.fontMetrics().boundingRect(self.text()).height()), self.text())
        painter.end()