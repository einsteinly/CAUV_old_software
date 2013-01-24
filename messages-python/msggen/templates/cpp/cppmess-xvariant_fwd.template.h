/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

