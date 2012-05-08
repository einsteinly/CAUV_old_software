/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_UTILITY_COORDINATES_H__
#define __CAUV_UTILITY_COORDINATES_H__

#include <cmath>

namespace cauv{

namespace coords{
// principal component of Earth along polar and equatorial axis (WGS84 coordinate system)
const double Earth_B = 6356752.314245; // polar
const double Earth_A = 6378137.00; // equatorial

double metresPerDegreeLongitude(double const& at_latitude_degrees);
double metresPerDegreeLatitude(double const& at_latitude_degrees);

} // namespace coords

} // namespace cauv

#endif // ndef __CAUV_UTILITY_COORDINATES_H__
