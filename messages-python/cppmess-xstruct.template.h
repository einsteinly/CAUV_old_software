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

/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${s.name.upper()}_H__
\#define __CAUV_${s.name.upper()}_H__

\#include <string>
\#include <vector>
\#include <list>
\#include <map>
\#include <boost/cstdint.hpp>
\#include <utility/streamops.h>
\#include <utility/serialisation-types.h>


#for $i in $includes
\#include ${i}
#end for

namespace cauv{

struct $s.name
{
    #for $f in $s.fields
    $toCPPType($f.type) $f.name;
    #end for
        
    ${s.name}();
    #if len($s.fields) > 0
    ${s.name}(#slurp
              #for i, f in $enumerate($s.fields)
#*           *#$toCPPType($f.type) const& $f.name#if $i < $len($s.fields) - 1#, #end if##slurp
              #end for
#*           *#);
    #end if 

    bool operator==($s.name const& other) const;
    bool operator<($s.name const& other) const;
};
void serialise(svec_ptr, $s.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $s.name&);
std::string chil($s.name const&);

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $s.name const& s)
{
    os << "$s.name {";
    #for i, f in $enumerate($s.fields)
    os << " $f.name = " << s.$f.name#if $i < $len($s.fields) - 1#<< ","#end if#;
    #end for
    os << " }";
    return os;
}

} // namespace cauv

\#endif//__CAUV_${s.name.upper()}_H__

