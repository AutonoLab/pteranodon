import datetime


class SensorData:
    def __init__(self, data=None):
        self.timestamp = datetime.datetime.now()
        self.data = data
        self.type = type(data)

    def update(self, data):
        self.timestamp = datetime.datetime.now()
        self.data = data
        self.type = type(data)

    @property
    def data(self):
        print("getting data")
        return self._data

    @data.setter
    def data(self, data):
        self._data = data

    @property
    def timestamp(self):
        print("getting time")
        return self._timestamp

    @timestamp.setter
    def timestamp(self, timestamp):
        self._timestamp = timestamp

    @property
    def type(self):
        print("getting type")
        return self._type

    @type.setter
    def type(self, data):
        self._type = type(data)
