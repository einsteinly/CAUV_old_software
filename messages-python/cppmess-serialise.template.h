/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_SERIALMESS_H__
\#define __CAUV_SERIALMESS_H__

\#include <utility/serialisation-types.h>
#for $e in $enums
\#include "${e.name}.h"
#end for
#for $v in $variants
\#include "${v.name}.h"
#end for

namespace cauv{

#for $s in $structs
struct $s.name;
#end for
#for $t in $included_types
$t.type $t.name;
#end for

#for $s in $structs
void serialise(svec_ptr, $s.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $s.name&);
#end for

#for $e in $enums
void serialise(svec_ptr, $e.name::e const&);
int32_t deserialise(const_svec_ptr, uint32_t, $e.name::e&);
#end for

#for $v in $variants
void serialise(svec_ptr, $v.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $v.name&);
#end for

#for $t in $included_types
void serialise(svec_ptr, $t.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $t.name&);
#end for

} // namespace cauv

\#endif // ndef __CAUV_SERIALMESS_H__

