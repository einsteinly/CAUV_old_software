/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_COMMON_MSG_CLASS_WGS84COORD_H__
#define __CAUV_COMMON_MSG_CLASS_WGS84COORD_H__

#include <stdexcept>

#include <generated/types/LatLongAlt.h>
#include "north_east_depth.h"

#include "utility/coordinates.h"

namespace cauv{

class WGS84Coord: public LatLongAlt{
    public:
        WGS84Coord()
            : LatLongAlt(){
        }

        WGS84Coord(LatLongAlt const& a)
            : LatLongAlt(a){
        }

        WGS84Coord(double const& deg_north, double const& deg_east, float const& altitude = 0.0f)
            : LatLongAlt(deg_north, deg_east, altitude){
        }

        static WGS84Coord fromDatum(const std::string& datum_name){
            // !!! TODO: these should be defined in a file and loaded at
            // runtime, or something, but this will do for now.
            // These are intended to match those defined in python
            // utils/coordinates.py
            if(datum_name == "river cam"){
                // Approximately! (Improve this when we've used the gps a bit more (bearing in
                // mind that's only ~15m accurate anyway). Altitude a complete guess...
                return WGS84Coord(52.116692, 0.117792, 30);
            }else if(datum_name == "NURC"){
                // Altitude unknown
                return WGS84Coord(44.095788, 9.865017, 0);
            }
            throw std::runtime_error("unknown datum: " + datum_name);
        }

        WGS84Coord& operator+=(NorthEastDepth const& ned){
            const double new_lat = latitude + ned.north() / coords::metresPerDegreeLatitude(latitude);
            const double new_lng = longitude + ned.east() / coords::metresPerDegreeLongitude(latitude);
            const double new_alt = altitude - ned.depth();
            return *this = WGS84Coord(new_lat, new_lng, new_alt);
        }

        WGS84Coord& operator-=(NorthEastDepth const& ned){
            const double new_lat = latitude - ned.north() / coords::metresPerDegreeLatitude(latitude);
            const double new_lng = longitude - ned.east() / coords::metresPerDegreeLongitude(latitude);
            const double new_alt = altitude + ned.depth();
            return *this = WGS84Coord(new_lat, new_lng, new_alt);
        }

        WGS84Coord operator+(NorthEastDepth const& ned) const{
            return WGS84Coord(*this) += ned;
        }

        WGS84Coord operator-(NorthEastDepth const& ned) const{
            return WGS84Coord(*this) -= ned;
        }
        
        // NB, the OPPOSITE of the python function differenceInMetresTo
        NorthEastDepth operator-(WGS84Coord const& other) const{
            const double mean_latitude = (latitude + other.latitude) / 2;
            const float north = (latitude - other.latitude) * coords::metresPerDegreeLatitude(mean_latitude);
            const float east = (longitude - other.longitude) * coords::metresPerDegreeLongitude(mean_latitude);
            const float depth = other.altitude - altitude;
            return NorthEastDepth(north, east, depth);
        }

};

} // namespace cauv

#endif // ndef __CAUV_COMMON_MSG_CLASS_WGS84COORD_H__

