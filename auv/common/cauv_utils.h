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

struct TimeStamp;
TimeStamp now();

uint16_t sumOnesComplement(std::vector<uint16_t> bytes);

#endif//__CAUV_UTILS_H__
