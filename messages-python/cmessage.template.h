\#ifndef __MESSAGES_H__
\#define __MESSAGES_H__

\#include <stdlib.h>
\#include <stdint.h>
\#include <string.h>

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
struct packet toPacket(struct Message* m);

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

struct $className* empty${className}();
struct $className* new${className}(#slurp
                                   #for i, f in $enumerate($m.fields)
#*                                *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                   #end for
#*                                *#);
struct ${className}* ${className}FromPacket(struct packet p);
struct packet ${className}ToPacket(struct $className* m);

#end for 
#end for 

// =============
// Serialisation
// =============
//
#for $t in $mapToBaseType($base_types)
#if $t.name == "string"
inline int load_string(struct packet p, int pos, char** val)
{
    uint32_t len;
    pos = load_uint32(p,pos,&len);
    *val = *(char**)(p.data+pos);
    return pos + len;
}
inline int save_string(struct packet p, int pos, char** val)
{
    uint32_t len = strlen(*val);
    pos = save_uint32(p,pos,&len);

    p.len += len;
    p.data = realloc(p.data, p.len);
    memcpy(*val, p.data+pos, len);
    return pos + len;
}
#else
inline int load_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    *val = *($toCType($t)*)(p.data+pos);
    return pos + sizeof($toCType($t));
}
inline int save_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    p.len += sizeof($toCType($t));
    p.data = realloc(p.data, p.len);
    *($toCType($t)*)(p.data+pos) = *val;
    return pos + sizeof($toCType($t));
}
#end if 
#end for 


#for $s in $structs
inline int load_${s.name}(struct packet p, int pos, $toCType($s)* s)
{
    #for $f in $s.fields
    pos = load_$loadsavesuffix($f.type)(p, pos, &s->$f.name);
    #end for
    return pos;
}
inline int save_${s.name}(struct packet p, int pos, $toCType($s)* s)
{
    #for $f in $s.fields
    pos = save_$loadsavesuffix($f.type)(p, pos, &s->$f.name);
    #end for
    return pos;
}
#end for

#for $e in $enums
inline int load_${e.name}(struct packet p, int pos, $toCType($e.type)* val)
{
    return load_$loadsavesuffix($e.type)(p, pos, val);
}
inline int save_${e.name}(struct packet p, int pos, $toCType($e.type)* val)
{
    return save_$loadsavesuffix($e.type)(p, pos, val);
}
#end for

#if $len($unknown_types) > 0
// Need to provide definitions of these manually.
#for $t in $unknown_types
int load_${t}(struct packet p, int pos, struct $t* val);
int save_${t}(struct packet p, int pos, struct $t* val);
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
