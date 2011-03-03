/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_CPPMESS_FWD_H__
\#define __CAUV_CPPMESS_FWD_H__

// standard integral types (int32_t etc)
\#include <boost/cstdint.hpp>
\#include <boost/variant/variant_fwd.hpp>

namespace cauv{

// =========================
// Forward Type Declarations
// =========================

#for $t in $unknown_types
class $t;
#end for

// enums can't be forward declared, so they are defined here
#for $e in $enums
namespace $e.name
{
    enum e
    {
        #for $i, $v in $enumerate($e.values)
        $v.name = $v.value,
        #end for
        NumValues = $len($e.values)
    };
} // namespace $e.name
#end for

#for $s in $structs
struct $s.name;
#end for

// sorry, can't forward declare boost variants :(
// (would need forward declarations of all types that could occur in the
// variant, including STL types, which is not possible

// ===============
// Message Classes
// ===============

namespace MessageType
{
    enum e
    {
        #set $num_values = 0
        #for $g in $groups
        #for $m in $g.messages
        $m.name = $m.id,
        #set $num_values += 1
        #end for
        #end for
        NumValues = $num_values
    };
} // namespace MessageType

// Base message class
class Message;

#for $g in $groups
// $g.name group
#for $m in $g.messages
#set $className = $m.name + "Message"
class $className;
#end for
#end for


// =======================
// Message Source/Observer
// =======================

class MessageObserver;
struct BufferingThreadBase;
class BufferedMessageObserver;
class DynamicObserver;
class DebugMessageObserver;
class UnknownMessageIdException;
class MessageSource;

} // namespace cauv

\#endif // ndef __CAUV_CPPMESS_FWD_H__
