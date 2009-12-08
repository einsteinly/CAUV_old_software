#ifndef __CAUV_UTILS_H__
#define __CAUV_UTILS_H__

#include <sstream>
using namespace std;

class make_string
{
    public:
        stringstream stream;
        operator string() const;

        template<class T>
        make_string& operator<<(T const& VAR) { stream << VAR; return *this; }
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

void milli_sleep(int milliseconds);

#endif//__CAUV_UTILS_H__
