import datetime


class SensorData:
    def __init__(self, data=None):
        self._timestamp = datetime.datetime.now()
        self._data = data
        self._type = type(data)

    def update(self, data):
        self._timestamp = datetime.datetime.now()
        self._data = data
        self._type = type(data)

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, data):
        self._data = data

    @property
    def timestamp(self):
        return self._timestamp

    @property
    def type(self):
        return self._type
