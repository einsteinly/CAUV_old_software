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

// Binary Serialisation
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
#if $t.superclass is None
void serialise(svec_ptr, $t.name const&);
int32_t deserialise(const_svec_ptr, uint32_t, $t.name&);
#end if
#end for

// CHIL serialisation
#for $s in $structs
std::string chil($s.name const&);
#end for

#for $e in $enums
std::string chil($e.name::e const&);
#end for

#for $v in $variants
std::string chil($v.name const&);
#end for

#for $t in $included_types
#if $t.superclass is None
std::string chil($t.name const&);
#end if
#end for

} // namespace cauv

\#endif // ndef __CAUV_SERIALMESS_H__

