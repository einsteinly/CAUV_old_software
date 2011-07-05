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
