#ifndef __CAUV_UTILS_H__
#define __CAUV_UTILS_H__

#include <boost/cstdint.hpp>
#include <sstream>
#include <vector>

#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

using namespace std;

class MakeString
{
    public:
        operator string() const;

        template<class T>
        MakeString& operator<<(T const& VAR) { stream << VAR; return *this; }
    
    protected:
        stringstream stream;
};

template <typename T, typename U> T convert_to(const U& in)
{
    return reinterpret_cast<const T&>(in);
}

template <typename T>
inline string to_string (const T& t)
{
	stringstream ss;
	ss << t;
	return ss.str();
}

void msleep(int milliseconds);

uint16_t sumOnesComplement(vector<uint16_t> bytes);

#endif//__CAUV_UTILS_H__
