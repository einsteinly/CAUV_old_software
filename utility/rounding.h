#ifndef __CAUV_UTILITY_ROUNDING_H__
#define __CAUV_UTILITY_ROUNDING_H__

#include <cmath>

template<typename T>
inline static T roundZ(T const& v){
    if(v >= 0) return std::floor(v);
    else return std::ceil(v);
}

template<typename T>
inline static T roundA(T const& v){
    if(v >= 0) return std::ceil(v);
    else return std::floor(v);
}

template<typename T>
inline static T round(T const& v){
    return std::floor(v+0.5);
}

template<typename T>
inline static T min(T const& a, T const& b){
    return a < b? a : b;
}

template<typename T>
inline static T max(T const& a, T const& b){
    return a > b? a : b;
}

template<typename TRet, typename TLow, typename TVal, typename THigh>
inline static TRet clamp_cast(TLow const& low, TVal const& value, THigh const& high){
    return value >= high? high : value <= low? low : TRet(value); 
}
template<typename TLow, typename TVal, typename THigh>
inline static TVal clamp(TLow const& low, TVal const& value, THigh const& high){
    return clamp_cast<TVal>(low, value, high);
}

#endif // ndef __CAUV_UTILITY_ROUNDING_H__

