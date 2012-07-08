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
\#ifndef __CAUV_${v.name}_FWD_H__
\#define __CAUV_${v.name}_FWD_H__

\#include <boost/variant/variant_fwd.hpp>

#for $i in $fwddecls
\#include ${i}
#end for

namespace cauv{

typedef boost::variant<
    #for $i, $t in $enumerate($v.types)
    #if $i < $len($v.types) -1
        $toCPPType($t),
    #else
        $toCPPType($t)
    #end if
    #end for
> $v.name;

} // namespace cauv

\#endif// __CAUV_${v.name}_FWD_H__

