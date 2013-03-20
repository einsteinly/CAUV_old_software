'''Python interface for controlling the SeaSprite and Gemini sonars.'''

# CAUV
import messaging
from debug import warning, debug

# Standard Library
from math import pi

class SeaSprite:
    def __init__(self, node):
        '''Python interface to the SeaSprite scanning sonar.'''        
        self.node = node
        self.current_direction = 0
        self.current_width = 6400
        self.current_gain = 255
        self.current_range = 50000
        self.current_range_res = 100
        self.current_angular_res = 16

    def direction(self, val):
        '''Set beam direction to 'val' native units (degrees*6400/360).'''
        self.current_direction = int(round(val))
        self.update()

    def directionDegrees(self, val):
        '''Set beam direction to 'val' degrees.'''
        self.current_direction = int(round(val * 6400 // 360))
        self.update()

    def directionRadians(self, val):
        '''Set beam direction to 'val' radians.'''
        self.current_direction = int(round(val * 6400 / pi))
        self.update()

    def width(self, val):
        '''Set scan-width in native units (degrees*6400/360).'''
        self.current_width = val
        self.update()

    def widthDegrees(self, val):
        '''Set scan-width in degrees.'''
        self.current_width = int(round(val * 6400 // 360))
        self.update()

    def widthRadians(self, val):
        '''Set scan-width in radians.'''
        self.current_width = int(round(val * 6400 / pi))
        self.update()

    def gain(self, val):
        '''Set the gain: possible values are 0--255.'''
        if val < 0 or val > 255:
            raise ValueError('Gain value must be in range 0--255.')
        self.current_gain = int(round(val))
        self.update()

    def range(self, val):
        '''Set range in mm. Possible values are 5000--100000'''
        if val <= 0:
            raise ValueError('Range must be > 0')
        self.current_range = int(round(val))
        self.update()

    def rangeRes(self, val):
        '''Set range resolution in mm.'''
        self.current_range_res = int(round(val))
        self.update()

    def angularRes(self, val):
        '''Set Angular resolution in native units (degrees*6400/360). Lower (higher-value) angular resolution means faster scans.'''
        self.current_angular_res = int(round(val))
        self.update()

    def angularResDegrees(self, val):
        '''Set Angular resolution in degrees. Lower (higher-value) angular resolution means faster scans.'''
        self.current_angular_res = val * 6400 // 360 # force integer division
        self.update()

    def angularResRadians(self, val):
        '''Set Angular resolution in radians. Lower (higher-value) angular resolution means faster scans.'''
        self.current_angular_res = int(round(val * 6400 / pi))
        self.update()

    def update(self):
        '''Send the current config to the sonar.'''        
        self.node.send(messaging.SonarControlMessage(
            self.current_direction,
            self.current_width,
            self.current_gain,
            self.current_range,
            self.current_range_res,
            self.current_angular_res
        ))

# compatibility
Sonar = SeaSprite

class Gemini:
    def __init__(self, node):
        '''Python interface to the Gemini multibeam sonar.'''
        self.node  = node
        self.current_range = 20 # m
        self.current_gain  = 60 # 0--100
        self.range_lines = 1000 # ~200 - ~8000?, about 1024 is sensible
        self.ping_continuous = False
        self.inter_ping_delay = 0.5 # seconds, can safely be set to zero
        self.priority = 0
        self.token = int(time.time() * 1000) & 0xffffffff
        info("Gemini Control Token: {}".format(self.token))
        self.timeout = timeout #seconds

    def get_token(self):
        return messaging.ControlLockToken(self.token, self.priority, self.timeout * 1000)
    
    def range(self, r):
        '''Set the range in metres (1--120); longer ranges produce lots more data but support lower frame-rates.'''
        self.current_range = r
        self.update()
    def gain(self, g):
        '''Set the gain (0--100), 40-70 are sensible values.'''
        self.current_gain = g
        self.update()
    def rangeLines(self, numLines):
        '''Set the number of returned range lines (spaced out evenly over 0--range metres) for gemini images.'''
        self.range_lines = numLines
        self.update()
    def continuous(self, ping_continuously):
        '''Ping repeatedly, with delay inter_ping_delay after receiving the end of each image.'''
        self.ping_continuous = ping_continuously
        self.update()
    def interPingDelay(self, inter_ping_delay):
        '''Set inter-ping delay (floating point seconds) for continuous pinging. Can safely be set to zero, or as high as you like.'''
        self.inter_ping_delay = inter_ping_delay
        self.update()
    def update(self):
        '''Send the current config to the sonar, triggers a ping if ping_continuously is not set.'''
        # now that the inter-ping delay is applied *after* the tail of the
        # previous ping, it can safely be set to zero
        #if self.ping_continuous and self.inter_ping_delay < 0.05 + 2.0 * self.current_range / 1400:
        #    warning('setting a dangerously low inter-ping value!')
        self.node.send(messaging.GeminiControlMessage(
            self.get_token(), self.current_range, self.current_gain, self.range_lines, self.ping_continuous, self.inter_ping_delay
        ))
    
    def __repr__(self):
        return 'range:%s gain:%s rangeLines:%s continuous:%s interPingDelay:%s' % (
            self.current_range, self.current_gain, self.range_lines, self.ping_continuous, self.inter_ping_delay
        )

