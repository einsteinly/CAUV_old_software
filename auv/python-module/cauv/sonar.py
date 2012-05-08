import messaging

from debug import warning, debug

from math import pi

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


class Gemini:
    def __init__(self, node):
        self.__node  = node
        self.__range = 20 # m
        self.__gain  = 60 # 0--100
        self.__range_lines = 1000 # ~200 - ~8000?, about 1024 is sensible
        self.__continuous = False
        self.__inter_ping_delay = 0.5 # seconds 
    
    def range(self, range):
        # set the range (metres, 1--120), longer ranges produce lots more data,
        # at lower frame-rates.
        self.__range = range
        self.update()
    def gain(self, gain):
        # set the gain (0--100), 40-70 are sensible values
        self.__gain = gain
        self.update()
    def rangeLines(self, numLines):
        # set the number of returned range lines (spaced out evenly over
        # 0--range metres) for gemini images
        self.__range_lines = numLines
        self.update()
    def continuous(self, ping_continuously):
        # ping continuously? If true, make sure that interPingDelay is set high
        # enough to allow the data to be returned!
        self.__continuous = ping_continuously
        self.update()
    def interPingDelay(self, inter_ping_delay):
        # set inter-ping delay (floating point seconds) for continuous pinging.
        self.__inter_ping_delay = inter_ping_delay
        self.update()
    def update(self):
        if self.__continuous and self.__inter_ping_delay < 0.05 + 2.0 * self.__range / 1400:
            warning('setting a dangerously low inter-ping value!')
        self.__node.send(messaging.GeminiControlMessage(
            self.__range, self.__gain, self.__range_lines, self.__continuous, self.__inter_ping_delay
        ))
    
    def __repr__(self):
        return 'range:%s gain:%s rangeLines:%s continuous:%s interPingDelay:%s' % (
            self.__range, self.__gain, self.__range_lines, self.__continuous, self.__inter_ping_delay
        )

