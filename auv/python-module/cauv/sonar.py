import messaging

class Sonar:
    def __init__(self, node):
        self.__node = node
        self._direction = 0
        self._width = 6400
        self._gain = 50
        self._range = 5000
        self._rangeRes = 10
        self._angularRes = 16

    def direction(self, val):
        self._direction = val
        self.update()

    def width(self, val):
        self._width = val
        self.update()

    def gain(self, val):
        self._gain = val
        self.update()

    def range(self, val):
        self._range = val
        self.update()

    def rangeRes(self, val):
        self._rangeRes = val
        self.update()

    def angularRes(self, val):
        self._angularRes = val
        self.update()

    def update(self):
        self.__node.send(messaging.SonarControlMessage(self._direction, self._width, self._gain, self._range, self._rangeRes, self._angularRes))

