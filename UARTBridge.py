from BridgeInterface import BridgeInterface as bi
import serial
# Concrete implemention of bridge using UART

ID = "/dev/ttyUSB0" # serial port of flight controller

class UARTBridge(bi):

    def __init__(self) -> None:
        super().__init__()
        self._serial = None

    # create serial object
    def initializeBridge(self):
        self._serial = serial.Serial()

    # attempt serial connection
    # return true if connection succeeds, false if connection fails
    def connectBridge(self):
        self._serial.port = ID
        try:
            self._serial.open()
            return True
        except:
            print('Error connecting to serial')
            return False 

    # close any open serial port
    def disconnectBridge(self):
        self._serial.close()

    # check serial port state
    @property
    def isConnected(self) -> bool:
        return self._serial.is_open

if __name__ == "__main__":
    uart = UARTBridge()
    uart.initializeBridge()
    print(uart.isConnected)
    uart.connectBridge()
    uart.disconnectBridge()
