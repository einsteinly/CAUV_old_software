
# Standard Library
import argparse
import threading
import copy
import math
import time

# CAUV
import cauv.messaging as msg
import cauv.node as node 
from cauv.debug import debug, warning, error, info
from utils import coordinates

# Third Party
import Image # PIL: MIT-like license
import numpy as np # BSD-type license

Default_Env_File = '/Users/james/Documents/IIB/Project/test_environment.png'
Default_Coordinates_Datum = coordinates.Simulation_Datum

def wrapAngleIntoPlusOrMinusPi(angle):
    while angle > math.pi:
        angle = angle - math.pi*2
    while angle < -math.pi:
        angle = angle + math.pi*2
    return angle

# Image origin is BOTTOM LEFT, rows (y) increase upwards (North), columns (x) increase
# to the right (East)
class Environment(object):
    def __init__(self, image_filename, origin_at_px_x_y, m_per_px, datum):
        self.image = Image.open(image_filename)
        # latitude/longitude position of world xyz coordinate system origin:
        self.datum = datum
        # origin in the image of the world xyz coordinate system origin:
        self.origin = origin_at_px_x_y
        self.m_per_px = float(m_per_px)

        debug('environment is %sx%s metres' %
            (self.image.size[0] * self.m_per_px,
             self.image.size[1] * self.m_per_px)
        )

    def llaToImageCoords(self, latlong):
        northeast = latlong.differenceInMetres(self.datum)
        x = northeast.east
        y = northeast.north
        return self.xyToImageCoords((x,y))

    def xyToImageCoords(self, xy):
        # xy in the same coordinate system as the image (y upwards, x right,
        # but in metres rather than pixels, and with a different origin)
        x = xy[0]; y = xy[1]
        return np.array((x / self.m_per_px - self.origin[0], y / self.m_per_px - self.origin[1]))
    
    def _boundingRect(self, pt_sequence):
        min = list(pt_sequence[0])
        max = list(pt_sequence[0])
        for pt in pt_sequence:
            if pt[0] < min[0]: min[0] = pt[0]
            if pt[0] > max[0]: max[0] = pt[0]
            if pt[1] < min[1]: min[1] = pt[1]
            if pt[1] > max[1]: max[1] = pt[1]
        return (min, max)
    
    def _getLineImageCoords(self, xy, start_angle, end_angle, length, res):
        # x,y:    in pixel-units from pixel origin
        # angle start/end:  measured in radians ccw from x-axis (note that a line at 90
        #         degrees points *negative* y in this system)
        # length: in px
        r = []
        assert(start_angle < end_angle)        
        start_angle = wrapAngleIntoPlusOrMinusPi(start_angle)
        end_angle = wrapAngleIntoPlusOrMinusPi(end_angle)
        
        def angleIsInRage(angle):
            if start_angle <= end_angle:
                return angle >= start_angle and angle <= end_angle
            else:
                return angle <= start_angle or angle >= end_angle

        start_direction = np.array((math.cos(start_angle), math.sin(start_angle)))
        end_direction   = np.array((math.cos(end_angle), math.sin(end_angle)))

        def isPxInImage(x,y):
            return x >= 0 and y > 0 and x < self.image.size[0] and y <= self.image.size[1]
        
        # these are significant optimisations!
        xy_x = xy[0]
        xy_y = xy[1]
        image_sizey = self.image.size[1]
        atan2 = math.atan2
        getpixel = self.image.getpixel

        for bin in xrange(0,res):
            support_px = [0.0]
            value = [0.0]
            #   
            #     - _ _
            #    |       - _ 
            #    |           - 
            # c4 |' - -        /
            #    |      ' .  /
            #    |__       / c2
            # c3 |  ''.. /
            #    |     / c1
            #    |   /
            #    | /
            #    o
            #
            #    y
            #    |
            #    o--> x
            #
            rmin = bin * float(length)/res
            rmax = (bin+1) * float(length)/res
            rmin_sq = rmin**2
            rmax_sq = rmax**2
            c1 = xy + start_direction * rmin
            c2 = xy + start_direction * rmax
            c3 = xy + end_direction * rmin
            c4 = xy + end_direction * rmax
            bbox_min, bbox_max = self._boundingRect((c1,c2,c3,c4))
            #print 'corners:', c1, c2, c3, c4
            ##print 'bbox:', bbox_min, bbox_max

            def pushPx(x, xlocal, xlocal_2, y):
                ylocal = y - xy_y
                r_sq = xlocal_2 + ylocal**2
                if r_sq >= rmin_sq and r_sq <= rmax_sq:
                    xy_angle = atan2(ylocal,xlocal)
                    if angleIsInRage(xy_angle):
                        support_px[0] += 1
                        # convert y into PIL coordinates: origin top
                        value[0] += getpixel(
                             (x, image_sizey - y)
                        )[0]
            
            #import dis
            #print dis.dis(pushPx)
            #sys.exit(0)
            
            for x in xrange(int(math.floor(bbox_min[0])),int(math.ceil(bbox_max[0]))):
                # more micro-optimisations... do I regret not writing this in c++? yes.
                xlocal = x - xy_x
                xlocal_2 = xlocal**2
                map(lambda y: pushPx(x, xlocal, xlocal_2, y),
                    filter(lambda y: isPxInImage(x,y),
                        xrange(int(math.floor(bbox_min[1])),int(math.ceil(bbox_max[1])))
                    )
                )

            if support_px[0] > 0:
                r.append(int(round(value[0]/support_px[0])))
            else:
                # no support? just sample the closest pixel:
                xy_mid = (c1 + c2 + c3 + c4) / 4
                x = round(xy_mid[0])
                y = round(xy_mid[1])
                if isPxInImage(x, y):
                    # convert y into PIL coordinates: origin top
                    r.append(getpixel((x, image_sizey - y))[0])
                else:
                    r.append(0)
        return r


    def getLine(self, latlong, bearing_start, bearing_end, length, res):
        # bearing start/end from north of line centre, degrees
        # length in real world metres
        # res in number-of-pixels
        image_xy = self.llaToImageCoords(latlong)
        angle_end = math.radians(90-bearing_start)
        angle_start = math.radians(90-bearing_end)
        return self._getLineImageCoords(
            image_xy, angle_start, angle_end, length / self.m_per_px, res
        )


class FakeGemini(msg.MessageObserver):
    def __init__(self, node, environment):
        msg.MessageObserver.__init__(self)
        self.node = node
        self.env = environment
        
        self._position_lock = threading.Lock()
        self._position = None

        self._config_lock = threading.Lock()
        self._config = None

        node.addObserver(self)
        node.subMessage(msg.SimPositionMessage())
        node.subMessage(msg.GeminiControlMessage())

    def position(self):
        self._position_lock.acquire()
        r = self._position
        self._position_lock.release()
        return r

    def setPosition(self, sim_position_message):
        self._position_lock.acquire()
        self._position = copy.deepcopy(sim_position_message)
        self._position_lock.release()

    def config(self):
        self._config_lock.acquire()
        r = self._config
        self._config_lock.release()
        return r

    def setConfig(self, gemini_control_message):
        self._config_lock.acquire()
        self._config = gemini_control_message
        self._config_lock.release()
    
    def gemBeamBearings(self):
        r = []
        for i in xrange(1,257):
            r.append(math.degrees(math.asin(((2.0 * i - 256) / 256.0) * 0.86602540)))
        return r

    def onSimPositionMessage(self, m):
        self.setPosition(m)
    
    def onGeminiControlMessage(self, m):
        self.setConfig(m)
        print m

    def mainLoop(self):
        while True:
            config = self.config()
            if not config:
                debug('no gemini config yet!')
                time.sleep(1)
                continue
            time.sleep(config.interPingPeriod)
            position = self.position()
            if not position:
                debug('no sim position yet!')
                continue

            #struct PolarImage
            #{
            #    data: list<byte>;
            #    // currently only raw uint8_t (isodistant responses in fastest varying index) is supported
            #    encoding: ImageEncodingType;
            #    // each bearing in range (-6400 to 6400) * 0x10000, these are the edges of
            #    // the bins that each iso-bearing beam of data falls into, so there are
            #    // num_beams + 1 values. The first bin is bearing_bins[0] --
            #    // bearing_bins[1], and so on.
            #    // Note that these are *bearings* not angles (ie, clockwise from forwards, not
            #    // anticlockwise from x axis (wherever that is...))
            #    bearing_bins: list<int32>;
            #    // number of bins = round(rangeEnd-rangeStart) / rangeConversion)
            #    rangeStart: float;
            #    rangeEnd: float;
            #    // rangeConversion = length of 1 bin in metres
            #    rangeConversion: float;
            #    // time of the ping from which the image is acquired
            #    timeStamp : TimeStamp;
            #}
            #enum SonarID: int8 { Seasprite = 1, Gemini = 2 }            
            #message SonarImage : 31
            #{
            #    source : SonarID;
            #    image : PolarImage;
            #}

            latlong = coordinates.LLACoord(position.latitude, position.longitude, position.altitude)

            beam_bearings = self.gemBeamBearings()

            bearing_bins = msg.floatVec()
            for i, a in enumerate(beam_bearings):
                if i == len(beam_bearings)-1:
                    prev_a = beam_bearings[i-1]
                    bearing_bins.append(a+(a-prev_a)/2)
                    break
                next_a = beam_bearings[i+1]
                if i == 0:
                    bearing_bins.append(a-(next_a-a)/2)
                bearing_bins.append(a+(next_a-a)/2)
            
            print 'at:', latlong, ' = pixels:', self.env.llaToImageCoords(latlong), 'look bearing:', position.orientation.yaw

            beams = []
            for i, a in enumerate(bearing_bins): 
                if i == len(bearing_bins)-1:
                    break
                beam_start = a
                beam_end = bearing_bins[i+1]
                assert(beam_end > beam_start)
                beams.append(self.env.getLine(
                    latlong,
                    position.orientation.yaw + beam_start,
                    position.orientation.yaw + beam_end,
                    config.range, config.rangeLines
                ))
            
            # oops, need to swap order...
            polar_data = msg.byteVec()
            for i in xrange(0, len(beams[0])):
                map(lambda x: polar_data.append(x[i]), beams)
            
            # map bearings from floating point bearing into the on-the-wire
            # integer format (see messages.msg)
            def mapTo64kRange(v):
                r = msg.int32Vec()
                for val in v:
                    i = round(val * 6400 * 0x10000 / 360.0);
                    if i >= 0:
                        i = i % (6400*0x10000);
                    else:
                        i = -((-i) % (6400*0x10000))
                    r.append(int(i))
                return r

            img = msg.PolarImage(
                polar_data,
                msg.ImageEncodingType.RAW_uint8_1,
                mapTo64kRange(bearing_bins),
                0.0,
                config.range,
                config.range / float(config.rangeLines),
                msg.now()
            )

            assert(round((config.range) / (config.range / float(config.rangeLines))) == len(beams[0]))

            self.node.send(msg.SonarImageMessage(msg.SonarID.Gemini, img))
            print '.'


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-e', '--environment', dest='env_file', default=Default_Env_File,
            help='Greyscale image to use as environment.')
    p.add_argument('-s', '--environment-scale', dest='env_scale', type=float, default=0.01,
            help='Metres per pixel in the environment file.')
    p.add_argument('-x', dest='env_x', type=float, default=0.0,
            help='x-origin of environment (px offset from top left)')
    p.add_argument('-y', dest='env_y', type=float, default=0.0,
            help='y-origin of environment (px offset from top left)')
    p.add_argument('-p', '--profile', dest='profile', default=False, action='store_true')

    # add a command line argument for this later...
    datum = Default_Coordinates_Datum
    
    opts, args = p.parse_known_args()
    cauv_node = node.Node("py-gem", args)
    
    try:
        e = Environment(opts.env_file, (opts.env_x, opts.env_y), opts.env_scale, datum)
        f = FakeGemini(cauv_node, e)
        if opts.profile:
            import cProfile as profile
            profile.run('f.mainLoop()', 'fakegemini.profile')
        else:
            f.mainLoop()
    finally:
        cauv_node.stop()

