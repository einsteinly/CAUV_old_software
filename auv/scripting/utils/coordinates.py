
# Standard Library Modules:
import math

# CAUV Modules:
import cauv.messaging as messaging 

#principal component of Earth along polar and equatorial axis
Earth_b = 6356752.3142 # polar
Earth_a = 6378137.00 # equatorial 

class NorthEastDepthCoord:
    #coordinates in metres North, East and Down w.r.t. to the reference corner
    def __init__(self, x, y, z):
        # z is positive depth
        self.north = x
        self.east = y
        self.depth = z
    def __repr__(self):
        return '(%s, %s, %s)' % (self.north, self.east, self.depth)

# See
# en.wikipedia.org/wiki/Geographic_coordinate_system#Expressing_latitude_and_longitude_as_linear_units
def metresPerDegreeLongitude(at_latitude_degrees):
    # dimensions of earth:
    tan_lat = math.tan(math.radians(at_latitude_degrees))
    beta = math.atan((Earth_b/Earth_a)*tan_lat)
    return abs(Earth_a * math.cos(beta) * math.radians(1))

def metresPerDegreeLatitude(at_latitude_degrees):
    return Earth_b * math.radians(1)

class LLACoord:
    def __init__(self, lat_deg, lng_deg, alt_m):
        # in degrees! altitude in metres above WGS84 ellipsoid
        self.latitude = lat_deg
        self.longitude = lng_deg
        self.altitude = alt_m
    def __add__(self, xyz):
        # add XYZ in metres onto the current latitude/longitude/altitude
        # z is positive altitude (-ve depth)
        if isinstance(xyz, messaging.floatXYZ):
            new_lat = self.latitude  + xyz.x / metresPerDegreeLatitude(self.latitude)
            new_lng = self.longitude + xyz.y / metresPerDegreeLongitude(self.latitude)
            new_alt = self.altitude + xyz.z
        else:
            new_lat = self.latitude  + xyz[0] / metresPerDegreeLatitude(self.latitude)
            new_lng = self.longitude + xyz[1] / metresPerDegreeLongitude(self.latitude)
            new_alt = self.altitude + xyz[2]
        return LLACoord(new_lat, new_lng, new_alt)
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

