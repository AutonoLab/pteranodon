from abc import abstractclassmethod, abstractproperty

# General interface between companion computer and flight controller
# Implement with your concrete variation (USB, Serial, Gazebo)

class BridgeInterface:

    def __init__(self) -> None:
        pass

    @abstractclassmethod
    def initializeBridge(self):
        pass

    @abstractclassmethod
    def connectBridge(self):
        pass

    @abstractclassmethod
    def disconnectBridge(self):
        pass