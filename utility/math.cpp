/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#include "math.h"
#include <cmath> // for floor

template<>
double cauv::mod(double const& d, double const& base)
{
    if(d > 0) {
        return d - base * std::floor(d / base);
    }else{
        return d + base * std::floor(-d / base);
    }
}

template<>
float cauv::mod(float const& d, float const& base)
{
    if(d > 0) {
        return d - base * std::floor(d / base);
    }else{
        return d + base * std::floor(-d / base);
    }
}
