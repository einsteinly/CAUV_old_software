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

#ifndef __CAUV_UTILITY_STRING_H__
#define __CAUV_UTILITY_STRING_H__

#include <sstream>
#include <string>

class MakeString{
    public:
        MakeString()
            : m_stream(){
        }
        
        MakeString(std::locale const& withLocale)
            : m_stream(){
            m_stream.imbue(withLocale);
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

template<typename T>
T fromStr(const char* v){
    std::istringstream s(v);
    T t;
    s >> t;
    return t;
}

#endif // ndef __CAUV_UTILITY_STRING_H__

