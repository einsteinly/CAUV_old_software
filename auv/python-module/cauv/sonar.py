import messaging

from math import pi

#pylint: disable=E1101

class Sonar:
    def __init__(self, node):
        self.__node = node
        self._direction = 0
        self._width = 6400
        self._gain = 50
        self._range = 5000
        self._rangeRes = 10
        self._angularRes = 16

    _exact_degree_ratio = 17 # 6400 / 360
    _exact_radian_ratio = 3200 / pi # 6400 / 2pi

    def set_direction_degrees(self, val):
        # updates direction to val degrees
        self._direction = self._exact_degree_ratio * val
        self.update()

    def set_direction_radians(self, val):
        # updates direction to val radians
        self._direction = self._exact_radian_ratio * val
        self.update()

    def set_width_degrees(self, val):
        # updates width to val degrees
        self._width = self._exact_degree_ratio * val
        self.update()

    def set_width_radians(self, val):
        # updates width to val radians
        self._width = self._exact_radian_ratio * val
        self.update()

    def set_gain(self, val):
        # val is an unsigned byte, [0, 255]
        self._gain = val
        self.update()

    def set_range(self, val):
        # val is in mm and non-negative
        self._range = val
        self.update()

    def set_rangeRes(self, val):
        # val is in mm and non-negative
        self._rangeRes = val
        self.update()

    def set_angularRes_degrees(self, val):
        # updates angularRes to val degrees
        self._angularRes = self._exact_degree_ratio * val
        self.update()

    def set_angularRes_radians(self, val):
        # updates angularRes to val radians
        self._angularRes = self._exact_radian_ratio * val
        self.update()

    def update(self):
        self.__node.send(messaging.SonarControlMessage(self._direction, self._width, self._gain, self._range, self._rangeRes, self._angularRes))

