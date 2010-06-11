\#include "cmesg.h"

// ===============
// Message Structs
// ===============

void freeMessage(struct Message* m)
{
}
char* toBytes(struct Message* m)
{
}

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
void free${className}(struct $className* m)
{

}
struct $className* empty${className}()
{

}
#if $len($m.fields) > 0
struct $className* new${className}(#slurp
                                   #for i, f in $enumerate($m.fields)
#*                                *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                   #end for
#*                                *#)
{

}
#end if
struct ${className}* ${className}FromPacket(struct packet p)
{

}
struct packet ${className}ToPacket(struct $className* m)
{

}

#end for 
#end for 


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m)
{

}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void send${className}(#slurp
                      #for i, f in $enumerate($m.fields)
#*                   *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                      #end for
#*                   *#)
{

}
#end for 
#end for 

// ================
// Message Handlers
// ================

void handlePacket(struct packet p)
{

}

// Implementations of handleXMessage are separate
