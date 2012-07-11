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

#ifndef __CAUV_STREAMOPS_MAP_H__
#define __CAUV_STREAMOPS_MAP_H__

#include <ostream>
#include <map>

namespace std{

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

} // namespace std

#endif // __CAUV_STREAMOPS_MAP_H__

