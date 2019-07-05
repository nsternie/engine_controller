"""
Base class for GUI objects. Used to define parameters all GUI objects need
"""

class BaseObject:

    def __init__(self, position: list, fluid: int, avionics_number: int = -1,
                 short_name: str = 'OX-SN-G07', safety_status: int = -1, long_name: str = 'LOX Dewar Drain',
                 is_vertical: bool = False):
        """
        Initializer for base class

        :param position: position of icon on screen
        :param fluid: fluid in object
        :param avionics_number: avionics identifier
        :param short_name: abbreviated name on schematics
        :param safety_status: safety criticality
        :param long_name: human-readable name for display on screen
        :param is_vertical: tracker if object is vertical
        """
        self.position = position
        self.fluid = fluid
        self.avionics_number = avionics_number
        self.short_name = short_name
        self.safety_status = safety_status
        self.long_name = long_name
        self.is_vertical = is_vertical