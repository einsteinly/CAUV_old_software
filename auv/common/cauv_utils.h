#ifndef __CAUV_UTILS_H__
#define __CAUV_UTILS_H__

#include <sstream>
#include <vector>

#include <boost/cstdint.hpp>

#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif


class MakeString
{
    public:
        MakeString() : stream() { }
        operator std::string() const;

        template<class T>
        MakeString& operator<<(T const& VAR) { stream << VAR; return *this; }

    protected:
        std::stringstream stream;
};

template <typename T, typename U> T convert_to(const U& in)
{
    return reinterpret_cast<const T&>(in);
}

template <typename T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}

template<typename T1, typename T2, typename T3>
inline static T2 clamp(T1 const& low, T2 const& a, T3 const& high){
    return (a < low)? low : ((a < high)? a : high);
}

struct TimeStamp;
TimeStamp now();

std::string now(std::string const& format);

uint16_t sumOnesComplement(std::vector<uint16_t> bytes);

void msleep(unsigned msecs);

template<typename T1, typename T2>
static T1 clamp(T1 const& low, T2 const& value, T1 const& high){
    return value > high? high : value < low? low : T1(value); 
}

#endif//__CAUV_UTILS_H__
