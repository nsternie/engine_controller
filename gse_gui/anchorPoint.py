from PyQt5.QtCore import *

class AnchorPoint(QPoint):
    """
    QPoint class that holds specific info to anchor points
    """

    def __init__(self, local_pos: QPoint, object_, x_aligned: bool = False, y_aligned: bool = False, parent=None):
        """
        Init for the AnchorPoint

        :param local_pos: local position of the point. Local means relative to the top left corner of the object
        :param object_: object point is assigned to
        :param x_aligned: is the x position of the point aligned with another objects anchor point
        :param y_aligned: is the y position of the point aligned with another objects anchor point
        :param parent: parent window
        """
        # Init the point
        super().__init__(local_pos.x() + object_.position.x(), local_pos.y() + object_.position.y())

        self.local_pos = local_pos
        self.object_ = object_
        self.x_aligned = x_aligned
        self.y_aligned = y_aligned
        self.parent = parent


    def updatePosition(self):
        """
        Updates the absolute position on the anchor point. Called when object moves
        """
        self.setX(self.local_pos.x() + self.object_.position.x())
        self.setY(self.local_pos.y() + self.object_.position.y())