import messaging

from math import pi

#pylint: disable=E1101

class Sonar:
    def __init__(self, node):
        self.__node = node
        self._direction = 0
        self._width = 6400
        self._gain = 255
        self._range = 50000
        self._rangeRes = 100
        self._angularRes = 16

    _exact_degree_ratio = 17 # 6400 / 360
    _exact_radian_ratio = 3200 / pi # 6400 / 2pi

    def direction(self, val):
        #update direction
        self._direction = val
        self.update()

    def directionDegrees(self, val):
        # updates direction to val degrees
        self._direction = val * 6400 // 360 # force integer division
        self.update()

    def directionRadians(self, val):
        # updates direction to val radians
        self._direction = int(round(val * 6400 / pi))
        self.update()

    def width(self, val):
        #update width
        self._width = val
        self.update()

    def widthDegrees(self, val):
        # updates width to val degrees
        self._width = val * 6400 // 360 # force integer division
        self.update()

    def widthRadians(self, val):
        # updates width to val radians
        self._width = int(round(val * 6400 / pi))
        self.update()

    def gain(self, val):
        # val is an unsigned byte, [0, 255]
        self._gain = val
        self.update()

    def range(self, val):
        # val is in mm and non-negative
        self._range = val
        self.update()

    def rangeRes(self, val):
        # val is in mm and non-negative
        self._rangeRes = val
        self.update()

    def angularRes(self, val):
        #update angularRes
        self._angularRes = val
        self.update()

    def angularResDegrees(self, val):
        # updates angularRes to val degrees
        self._angluarRes = val * 6400 // 360 # force integer division
        self.update()

    def angularResRadians(self, val):
        # updates angularRes to val radians
        self._angularRes = int(round(val * 6400 / pi))
        self.update()

    def update(self):
        self.__node.send(messaging.SonarControlMessage(self._direction, self._width, self._gain, self._range, self._rangeRes, self._angularRes))

