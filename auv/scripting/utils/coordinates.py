
# Standard Library Modules:
import math

# CAUV Modules:
import cauv.messaging as messaging 

#principal component of Earth along polar and equatorial axis (WGS84 coordinate system)
Earth_b = 6356752.314245 # polar
Earth_a = 6378137.00 # equatorial 


class NorthEastDepthCoord:
    #coordinates in metres North, East and Down w.r.t. to the reference corner
    def __init__(self, north, east, depth):
        # z is positive depth
        self.north = north
        self.east = east
        self.depth = depth
    def __mul__(self, number):
        return NorthEastDepthCoord(self.north * number,
                                   self.east * number,
                                   self.depth * number)
    def __div__(self, number):
        return NorthEastDepthCoord(self.north / number,
                                   self.east / number,
                                   self.depth / number)
    def __sub__(self, ned):
        if isinstance(ned, NorthEastDepthCoord):
            return NorthEastDepthCoord(self.north - ned.north,
                                       self.east - ned.east,
                                       self.depth - ned.depth)
        else:
            raise TypeError('Subtracting %s from a NorthEastDepthCoord is unsupported' %str(type(ned)))
        return self
    def __imul__(self, number):
        self.north *= number
        self.east *= number
        self.depth *= number
        return self
    def __idiv__(self, number):
        self.north /= number
        self.east /= number
        self.depth /= number
        return self
    def __iadd__(self, ned):
        if isinstance(ned, NorthEastDepthCoord):
            self.north += ned.north
            self.east += ned.east
            self.depth += ned.depth
        else:
            raise TypeError('Adding %s to a NorthEastDepthCoord is unsupported' %str(type(ned)))
        return self
    def __repr__(self):
        return '(North=%s, East=%s, Depth=%s)' % (self.north, self.east, self.depth)

# See
# en.wikipedia.org/wiki/Geographic_coordinate_system#Expressing_latitude_and_longitude_as_linear_units
def metresPerDegreeLongitude(at_latitude_degrees):
    # dimensions of earth:
    tan_lat = math.tan(math.radians(at_latitude_degrees))
    beta = math.atan((Earth_b/Earth_a)*tan_lat)
    return abs(Earth_a * math.cos(beta) * math.radians(1))

def metresPerDegreeLatitude(at_latitude_degrees):
    # TODO maybe improve approx; atm 1st degree    
    return Earth_b * math.radians(1)

class LLACoord:
    def __init__(self, lat_deg, lng_deg, alt_m):
        # in degrees north & EAST! altitude in metres above WGS84 ellipsoid
        self.latitude = lat_deg  # degrees north WGS84
        self.longitude = lng_deg # degrees east WGS84
        self.altitude = alt_m    # metres above WGS84 ellipsoid
    @classmethod
    def fromWGS84(cls, wgs84):
        return cls(wgs84.latitude, wgs84.longitude, wgs84.altitude)
    def toWGS84(self):
        return messaging.WGS84Coord(self.latitude, self.longitude, self.altitude)
    def __add__(self, ned):
        # add NorthEastDepth coord (a cartesian vector) 'ned' to the current latitude/longitude
        if isinstance(ned, NorthEastDepthCoord):
            new_lat = self.latitude + ned.north / metresPerDegreeLatitude(self.latitude)
            new_lng = self.longitude + ned.east / metresPerDegreeLongitude(self.latitude)
            new_alt = self.altitude - ned.depth
        else:
            raise Exception('you can\'t add XYZ to lat/long without specifying'
                'a coordinate system! You should use a NorthEastDepthCoord'
                'instead'
            )
            xyz = ned
            new_lat = self.latitude  + xyz[1] / metresPerDegreeLatitude(self.latitude)
            new_lng = self.longitude + xyz[0] / metresPerDegreeLongitude(self.latitude)
            new_alt = self.altitude + xyz[2]
        return LLACoord(new_lat, new_lng, new_alt)

    def differenceInMetresTo(self, lla_coord):
        # subtract this from another LLACoord, returning the result as a
        # NorthEastDepthCoord (in metres). Sign is that relative to this, i.e.
        # if lla_coord is north of this, r.north will be positive
        mean_latitude = (self.latitude + lla_coord.latitude) / 2
        north = (lla_coord.latitude - self.latitude) * \
            metresPerDegreeLatitude(mean_latitude)
        east = (lla_coord.longitude - self.longitude) * \
            metresPerDegreeLongitude(mean_latitude)
        depth = self.altitude - lla_coord.altitude
        return NorthEastDepthCoord(north, east, depth)

    def __repr__(self):
        return '(%s, %s, %s)' % (self.latitude, self.longitude, self.altitude)



# Normally we operate in xyz coordinates in a local area around a
# latitude/longitude coordinate, these defines the base coordinates that we've
# used:

# Approximately! (Improve this when we've used the gps a bit more (bearing in
# mind that's only ~15m accurate anyway). Altitude a complete guess...
River_Cam_Datum = LLACoord(52.116692, 0.117792, 30)

# Altitude unknown
NURC_Harbour_Datum = LLACoord(44.095788, 9.865017, 0)

# just for GUI display during sim, really:
Simulation_Datum = River_Cam_Datum;

