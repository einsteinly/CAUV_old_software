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

#ifndef __CAUV_MATH_H__
#define __CAUV_MATH_H__

#include <cmath>

namespace cauv {

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

}

#endif//__CAUV_MATH_H__
