#ifndef __CAUV_UTILITY_STRING_H__
#define __CAUV_UTILITY_STRING_H__

#include <sstream>
#include <string>

class MakeString{
    public:
        MakeString()
            : m_stream(){
        }
        
        template<typename T>
        MakeString(T const& v)
            : m_stream(){
            m_stream << v;
        }

        operator std::string() const{
            return m_stream.str();
        }

        template<class T>
        MakeString& operator<<(T const& v){
            m_stream << v;
            return *this;
        }

    protected:
        std::stringstream m_stream;
};

typedef MakeString mkStr;

template<typename T>
std::string toStr(T const& v){
    return mkStr() << v;
}

#endif // ndef __CAUV_UTILITY_STRING_H__

