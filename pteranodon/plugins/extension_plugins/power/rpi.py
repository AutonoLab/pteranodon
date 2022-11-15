class RPi:
    """
    A class for acquring the power usage of a Raspberry pi 4
    """

    def __init__(self):
        try:
            raise NotImplementedError
        except NotImplementedError as err:
            raise NotImplementedError("RPi not implemented yet") from err

    @staticmethod
    def get_current_power() -> int:
        """Get current power draw"""

        return -1

    @staticmethod
    def get_power_used() -> int:
        """Get lifetime power draw"""

        return -1
