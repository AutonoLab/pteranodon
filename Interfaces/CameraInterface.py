from abc import ABC, abstractmethod


class CameraInterface(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def getFrame(self):
        pass
