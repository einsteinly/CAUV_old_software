/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_MATH_H__
#define __CAUV_MATH_H__

#include <cmath>

namespace cauv {
    
    /* These modulus functions return the closest possible value to zero, and
     * can be used safely with floating point numbers.
     */

    template<typename T>
    T mod(T const& d, T const& base)
    {
        T ret = d % base;
        if (ret < 0)
            ret += base;
        return ret;
    }
   
    template<>
    double mod(double const& d, double const& base);
    
    template<>
    float mod(float const& d, float const& base);
    
    template<typename T>
    T degrees(T const& radians){
        return radians * 180 / 3.14159265358979323846;
    }

    template<typename T>
    T radians(T const& degrees){
        return degrees * 3.14159265358979323846 / 180;
    }
}

#endif//__CAUV_MATH_H__
