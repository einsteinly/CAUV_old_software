/***  This is a generated file, do not edit ***/
\#ifndef __MESSAGE_FWD_H__
\#define __MESSAGE_FWD_H__

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
class BufferingThreadBase;
class BufferedMessageObserver;
class DynamicObserver;
class DebugMessageObserver;
class UnknownMessageIdException;
class MessageSource;

\#endif//__MESSAGE_FWD_H__
