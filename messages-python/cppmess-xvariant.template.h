/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_${v.name}_H__
\#define __CAUV_${v.name}_H__

\#include <boost/variant.hpp>
\#include <utility/streamops.h>
\#include <utility/serialisation-types.h>

#for $i in $includes
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

void serialise(svec_ptr, $v.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $v.name&);

} // namespace cauv

\#endif// __CAUV_${v.name}_H__

