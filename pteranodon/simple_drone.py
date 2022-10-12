from .abstract_drone import AbstractDrone


class SimpleDrone(AbstractDrone):
    """
    A bare minimum "drone" with all methods filled with pass for syntax purposes
    """

    def __init__(self, address: str):
        super().__init__(address=address)

    def setup(self):
        pass

    def start_loop(self):
        pass

    def loop(self):
        pass

    def teardown(self):
        pass
