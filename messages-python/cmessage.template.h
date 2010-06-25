\#ifndef __MESSAGES_H__
\#define __MESSAGES_H__

\#include <stdlib.h>
\#include <string.h>

// ================
// Type definitions
// ================

typedef signed char int8_t;
typedef signed int int16_t;
typedef signed long int32_t;
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;
typedef unsigned long uint32_t;

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
#for $v in $e.values
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
void destroyPacket(struct packet* p);

// Base message struct
struct Message
{
    uint32_t id;
};
void sendAsPacket(struct Message* m);

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
struct $className
{
    uint32_t id;
    
    #for $f in $m.fields
    $toCType($f.type) ${f.name};
    #end for     
};

struct $className empty${className}();
struct $className new${className}(#slurp
                                   #for i, f in $enumerate($m.fields)
#*                                *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                   #end for
#*                                *#);
struct ${className} ${className}FromPacket(struct packet p);
void send${className}AsPacket(struct $className* m);

#end for 
#end for 

// =============
// Serialisation
// =============
//
#for $t in $mapToBaseType($base_types)
int load_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val);
int save_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val);
int len_$loadsavesuffix($t)($toCType($t)* val);
#end for 


#for $s in $structs
int load_${s.name}(struct packet p, int pos, $toCType($s)* s);
int save_${s.name}(struct packet p, int pos, $toCType($s)* s);
int len_${s.name}($toCType($s)* s);
#end for

#for $e in $enums
int load_${e.name}(struct packet p, int pos, $toCType($e.type)* val);
int save_${e.name}(struct packet p, int pos, $toCType($e.type)* val);
int len_${e.name}($toCType($e.type)* val);
#end for

#if $len($unknown_types) > 0
// Need to provide definitions of these manually.
#for $t in $unknown_types
int load_${t}(struct packet p, int pos, struct $t* val);
int save_${t}(struct packet p, int pos, struct $t* val);
int len_${t}(struct $t* val);
#end for
#end if 

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
