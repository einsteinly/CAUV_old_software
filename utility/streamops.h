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

#ifndef __CAUV_STREAMOPS_H__
#define __CAUV_STREAMOPS_H__

#include <ostream>
#include <vector>
#include <map>
#include <list>
#include <set>
#include <utility>

namespace std{

template<typename T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::vector<T> const& a){
    os << "vec[" << a.size() << "] {";
    for(typename std::vector<T>::const_iterator i = a.begin(); i != a.end();){
        os << *i;
        if(++i != a.end())
            os << ", ";
    }
    os << "}";
    return os;
}

template<typename T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::list<T> const& a){
    os << "list[" << a.size() << "] {";
    for(typename std::list<T>::const_iterator i = a.begin(); i != a.end();){
        os << *i;
        if(++i != a.end())
            os << ", ";
    }
    os << "}";
    return os;
}

template<typename key_T, typename val_T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::map<key_T, val_T> const& m){
    os << "map[" << m.size() << "] {";
    for(typename std::map<key_T, val_T>::const_iterator i = m.begin(); i != m.end();){
        os << i->first << " : " << i->second;
        if(++i != m.end())
            os << ", ";
    }
    os << "}";
    return os;
}

template<typename T, typename Comp, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::set<T, Comp> const& s){
    os << "set[" << s.size() << "] {";
    for(typename std::set<T, Comp>::const_iterator i = s.begin(); i != s.end();){
        os << *i;
        if(++i != s.end())
            os << ", ";
    }
    os << "}";
    return os;
}

template<typename T1, typename T2, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::pair<T1, T2> const& a){
    os << "{" << a.first << "," << a.second << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_H__

