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

#ifndef __CAUV_STREAMOPS_LIST_H__
#define __CAUV_STREAMOPS_LIST_H__

#include <ostream>
#include <list>

namespace std{

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

} // namespace std

#endif // __CAUV_STREAMOPS_LIST_H__

