\#ifndef __MESSAGES_H__
\#define __MESSAGES_H__

\#include <stdlib.h>
\#include <stdint.h>

// ================
// Type definitions
// ================

#for $t in $unknown_types
struct $t;
#end for

#for $s in $structs
struct $s.name
{
    #for $f in $s.fields
    $toCType($f.type) $f.name;
    #end for
};
#end for

#for $e in $enums
#for $i, $v in $enumerate($e.values)
#define CAUV_$($e.name.upper())_$v.name.upper() $v.value
#end for
#define CAUV_$($e.name.upper())_NUMVALUES $len($e.values)

#end for


// ===============
// Message Structs
// ===============

struct packet {
    unsigned char* data;
    uint32_t length;
};

// Base message struct
struct Message
{
    uint32_t id;
    char* group;
    void (*destructor)(struct Message* m);
    void (*toBytes)(struct Message* m);
};
void freeMessage(struct Message* m);
char* toBytes(struct Message* m);

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
struct $className
{
    uint32_t id;
    char* group;
    void (*destructor)(struct Message* m);
    void (*toBytes)(struct Message* m);
    
    #for $f in $m.fields
    $toCType($f.type) ${f.name};
    #end for     
};
void free${className}(struct $className* m);
struct $className* empty${className}();
#if $len($m.fields) > 0
struct $className* new${className}(#slurp
                                   #for i, f in $enumerate($m.fields)
#*                                *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                   #end for
#*                                *#);
#end if
struct ${className}* ${className}FromPacket(struct packet p);
struct packet ${className}ToPacket(struct $className* m);

#end for 
#end for 


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m);

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void send${className}(#slurp
                      #for i, f in $enumerate($m.fields)
#*                   *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                      #end for
#*                   *#);
#end for 
#end for 

// ================
// Message Handlers
// ================

void handlePacket(struct packet p);

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void handle${className}(struct $className* m);
#end for 
#end for 


\#endif//__MESSAGES_H__
