from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from constants import Constants

class ControlsPanelWidget(QWidget):
    """
    Widget that contains controls that are not through icons on screen. Ex. Editing, Arming etc
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.window = parent
        # TODO: Rename the controls because it is weird
        self.controls = self.window.controlsWidget

        self.gui = self.window.parent

        # Keeps track of all the objects currently being edited
        self.objects_editing = []

        self.left = self.gui.screenResolution[0] - self.window.panel_width
        self.top = 0

        self.width = self.window.panel_width
        self.height = self.gui.screenResolution[1]
        self.setGeometry(self.left, self.top, self.width, self.height)

        # Sets the color of the panel to dark Gray
        # TODO: Make this not look totally terrible
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkGray)
        self.setPalette(p)

        # Frames and layouts that holds everything in it and can be hidden / shown
        self.edit_frame = QFrame(self)
        self.edit_form_layout = QFormLayout(self)
        self.edit_frame.setLayout(self.edit_form_layout)

        # Textboxes, radioButtons, and drop-downs
        self.long_name_textbox = QLineEdit(self)
        self.label_position_combobox = QComboBox(self)
        self.avionics_number_textbox = QLineEdit(self)
        self.fluid_combobox = QComboBox(self)
        self.short_name_textbox = QLineEdit(self)
        self.scale_spinbox = QDoubleSpinBox(self)

        # Inits above widgets
        self.initEditFrame()

        self.show()

    def initEditFrame(self):
        """
        Inits the widgets inside of the editing frame
        """

        # Add text boxes to the layout
        self.createTFRadioButtons("Long Name Label", "Enabled", "Disabled", True)
        self.createLineEdit(self.long_name_textbox, "Long Name")
        self.createComboBox(self.label_position_combobox, "Label Position", ["Top","Right","Bottom","Left"])
        self.createLineEdit(self.avionics_number_textbox, "Avionics Number", QIntValidator())
        self.createComboBox(self.fluid_combobox, "Fluid", Constants.fluids)
        self.createTFRadioButtons("Position is", "Locked", "Unlocked", False)
        self.createLineEdit(self.short_name_textbox, "Short Name")
        self.createSpinbox(self.scale_spinbox, "Scale", .1, 10, 0.1)

        self.edit_frame.hide()

    def createLineEdit(self, lineEdit: QLineEdit, identifier, validator: QValidator = None):
        """
        Creates text box user can type in, used for editing labels and so forth
        :param lineEdit: reference to line edit widget
        :param identifier: identifies line edit widget, used when text changes
        :param validator: validator used to make sure text entered is int, double etc.
        """
        identifier_label = QLabel(identifier + ":")
        lineEdit.textChanged.connect(lambda : self.updateEditingObjectFields(lineEdit.text(), identifier))
        if validator is not None:
            lineEdit.setValidator(validator)

        self.edit_form_layout.addRow(identifier_label, lineEdit)

    # TODO: Make this cleaner, kinda messy right now
    def createTFRadioButtons(self, identifier, true_btn_label = "True", false_btn_label = "False", checked:bool = True):
        """
        Creates two radio buttons to enable/disable aspects of the controls widget
        :param identifier: identifies radioButton widget, used when button is toggled
        :param true_btn_label: label on the button that correlates to True
        :param false_btn_label: label on the button that correlates to False
        :param checked: Which button, True or False, is initially checked
        """
        identifier_label = QLabel(identifier + ":")

        group = QButtonGroup(self)

        hbox = QHBoxLayout()
        true_button = QRadioButton(true_btn_label)
        false_button = QRadioButton(false_btn_label)

        if checked:
            true_button.setChecked(True)
        else:
            false_button.setChecked(True)

        group.addButton(true_button)
        group.addButton(false_button)

        true_button.toggled.connect(lambda: self.updateEditingObjectFields(True, identifier))
        false_button.toggled.connect(lambda: self.updateEditingObjectFields(False, identifier))

        hbox.addWidget(true_button)
        hbox.addWidget(false_button)
        hbox.addStretch()

        self.edit_form_layout.addRow(identifier_label, hbox)

    def createComboBox(self, comboBox: QComboBox, identifier:str, items: []):
        """
        Created a drop-down box for user selection
        :param comboBox: reference to combobox
        :param identifier: identifies comboBox widget, used when comboBox index changes
        :param items: list of strings user can select in drop-down
        """
        identifier_label = QLabel(identifier + ":")

        comboBox.setFixedWidth(100)
        comboBox.addItems(items)
        comboBox.currentIndexChanged.connect(lambda: self.updateEditingObjectFields(comboBox.currentText(), identifier))

        self.edit_form_layout.addRow(identifier_label, comboBox)

    def createSpinbox(self, spinBox: QSpinBox, identifier:str, min:int = 0, max:int = 100, step:float = 1):

        identifier_label = QLabel(identifier + ":")
        spinBox.setMinimum(min)
        spinBox.setMaximum(max)
        spinBox.setSingleStep(step)

        spinBox.valueChanged.connect(lambda: self.updateEditingObjectFields(spinBox.value(), identifier))

        self.edit_form_layout.addRow(identifier_label, spinBox)


    def updateEditPanelFields(self, object_):
        """
        Updates the various fields in the edit frame when a new object is selected for editing
        """
        self.long_name_textbox.setText(object_.long_name)
        self.label_position_combobox.setCurrentIndex(object_.long_name_label_position_num)
        self.avionics_number_textbox.setText(str(object_.avionics_number))
        self.fluid_combobox.setCurrentText(Constants.fluid[object_.fluid])
        self.short_name_textbox.setText(object_.short_name)
        self.scale_spinbox.setValue(object_.scale)

        self.avionics_number_textbox.setDisabled(True)

    def updateEditingObjectFields(self, text, identifier):
        """
        Called when user changes field of an object from the edit panel
        :param text: Text of field that is being changed
        :param identifier: identifier of the field being changed
        """
        # Gets the object being edited right now and updated the fields based on identifier
        for object_ in self.objects_editing:
            if object_.is_being_edited:
                if identifier == "Long Name":
                    object_.setLongName(text)
                elif identifier == "Label Position":
                    # TODO: Fix the badness
                    if text == "Top":
                        object_.setLongNameLabelPosition(0)
                    elif text == "Right":
                        object_.setLongNameLabelPosition(1)
                    elif text == "Bottom":
                        object_.setLongNameLabelPosition(2)
                    elif text == "Left":
                        object_.setLongNameLabelPosition(3)
                elif identifier == "Avionics Number":
                    if text != "":
                        object_.setAvionicsNumber(int(text))
                elif identifier == "Fluid":
                    object_.setFluid(text)
                elif identifier == "Long Name Label":
                    object_.long_name_label.setVisible(text)
                elif identifier == "Position is":
                    object_.setPositionLock(text)
                elif identifier == "Short Name":
                    object_.setShortName(text)
                elif identifier == "Scale":
                    object_.setScale(text)

    # FIXME: Things don't work well if more than one object are in here
    # HMM: Move is_being_edited to object class and call this function here
    def addEditingObjects(self, objects):
        """
        Adds object to list to be edited
        """
        objects.is_being_edited = True
        self.objects_editing.append(objects)
        self.edit_frame.show()
        self.updateEditPanelFields(objects)

    def removeEditingObjects(self, objects):
        """
        Removes object from list to be edited
        """
        objects.is_being_edited = False
        self.objects_editing.remove(objects)

        # If no objects are being edited hide the edit frame
        if len(self.objects_editing) == 0:
            self.edit_frame.hide()

    def removeAllEditingObjects(self):
        """
        Sets all objects to not be editing and clears the editing list
        """
        for object_ in self.objects_editing:
            object_.is_being_edited = False

        self.objects_editing.clear()
        self.edit_frame.hide()

    def save(self):
        """
        This saves the edits and changes back into user mode
        """
        self.removeAllEditingObjects()






