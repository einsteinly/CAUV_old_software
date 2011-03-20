/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_SERIALMESS_H__
\#define __CAUV_SERIALMESS_H__

\#include <generated/messages_fwd.h>
\#include <generated/messages.h>

\#include <utility/serialisation-types.h>

namespace cauv{

#for $t in $unknown_types
void serialise(svec_ptr, $t const&);
int32_t deserialise(const_svec_ptr, int32_t, $t&);
#end for
#for $e in $enums
void serialise(svec_ptr, $e.name::e const&);
int32_t deserialise(const_svec_ptr, int32_t, $e.name::e&);
#end for
#for $s in $structs
void serialise(svec_ptr, $s.name const&);
int32_t deserialise(const_svec_ptr, int32_t, $s.name&);
#end for
#for $v in $variants
void serialise(svec_ptr, $v.name const&);
int32_t deserialise(const_svec_ptr, int32_t, $v.name&);
#end for
// Message serialisation (deserialisation handled lazily in messages.cpp)
#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void serialise(svec_ptr p, $className const& v);
#end for
#end for

} // namespace cauv

\#endif // ndef __CAUV_SERIALMESS_H__

