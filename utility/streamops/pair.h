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

#ifndef __CAUV_STREAMOPS_PAIR_H__
#define __CAUV_STREAMOPS_PAIR_H__

#include <ostream>
#include <utility>

namespace std{

template<typename T1, typename T2, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::pair<T1, T2> const& a){
    os << "{" << a.first << "," << a.second << "}";
    return os;
}

} // namespace std

#endif // __CAUV_STREAMOPS_PAIR_H__

