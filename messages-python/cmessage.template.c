\#include "${headerFile}"

// ===============
// Message Structs
// ===============

struct packet toPacket(struct Message* m)
{
    switch (m->id) {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
            return ${className}ToPacket((struct $className*)m);
        #end for 
        #end for
    }
    {
        struct packet p;
        return p;
    }
}

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
struct $className empty${className}()
{
    struct $className m;
    m.id = $m.id;
    return m;
}
struct $className new${className}(#slurp
                                  #for i, f in $enumerate($m.fields)
#*                               *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                  #end for
#*                               *#)
{
    struct $className m = empty${className}();
    #for $f in $m.fields
    m.$f.name = $f.name;
    #end for
    return m;
}
struct ${className} ${className}FromPacket(struct packet p)
{
    struct $className m = empty${className}();
    int pos = 4; // Skip the message id (not as safe, but faster)

    #for $f in $m.fields
    pos = load_$loadsavesuffix($f.type)(p, pos, &m.$f.name);
    #end for

    return m;
}
struct packet ${className}ToPacket(struct $className* m)
{
    int msgsize = 0;
    #for $f in $m.fields
    msgsize = getsize_$loadsavesuffix($f.type)(p, pos, &m->$f.name);
    #end for

    unsigned char bytes[msgsize];
    struct packet p;
    p.data = bytes;
    p.length = msgsize;

    int pos = 0;

    pos = save_uint32(p, pos, &m->id);

    #for $f in $m.fields
    pos = save_$loadsavesuffix($f.type)(p, pos, &m->$f.name);
    #end for

    return p;
}

#end for 
#end for 


// =============
// Serialisation
// =============
//
#for $t in $mapToBaseType($base_types)
#if $t.name == "string"
int load_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    uint32_t len;
    pos = load_uint32(p,pos,&len);
    *val = *(char**)(p.data+pos);
    return pos + len;
}
int save_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    uint32_t len = strlen(*val);
    pos = save_uint32(p,pos,&len);
    memcpy(*val, p.data+pos, len);
    return pos + len;
}
int len_$loadsavesuffix($t)($toCType($t)* val)
{
    uint32_t len = strlen(*val);
    return len_uint32(len) + len;
}
#else
int load_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    *val = *($toCType($t)*)(p.data+pos);
    return pos + sizeof($toCType($t));
}
int save_$loadsavesuffix($t)(struct packet p, int pos, $toCType($t)* val)
{
    *($toCType($t)*)(p.data+pos) = *val;
    return pos + sizeof($toCType($t));
}
int len_$loadsavesuffix($t)($toCType($t)* val)
{
    return sizeof($toCType($t));
}
#end if 
#end for 


#for $s in $structs
int load_${s.name}(struct packet p, int pos, $toCType($s)* s)
{
    #for $f in $s.fields
    pos = load_$loadsavesuffix($f.type)(p, pos, &s->$f.name);
    #end for
    return pos;
}
int save_${s.name}(struct packet p, int pos, $toCType($s)* s)
{
    #for $f in $s.fields
    pos = save_$loadsavesuffix($f.type)(p, pos, &s->$f.name);
    #end for
    return pos;
}
int len_${s.name}($toCType($s)* s)
{
    size = 0;
    #for $f in $s.fields
    size += len_$loadsavesuffix($f.type)(p, size, &s->$f.name);
    #end for
    return size;
}
#end for

#for $e in $enums
int load_${e.name}(struct packet p, int pos, $toCType($e.type)* val)
{
    return load_$loadsavesuffix($e.type)(p, pos, val);
}
int save_${e.name}(struct packet p, int pos, $toCType($e.type)* val)
{
    return save_$loadsavesuffix($e.type)(p, pos, val);
}
int len_${e.name}($toCType($e.type)* val)
{
    return len_$loadsavesuffix($e.type)(p, size, val);
}
#end for


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m)
{
    struct packet p = toPacket(m);
    sendPacket(p);
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void send${className}(#slurp
                      #for i, f in $enumerate($m.fields)
#*                   *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                      #end for
#*                   *#add_subdirectory (webcam)
)
{
    struct $className m = new${className}(#slurp
                                          #for i, f in $enumerate($m.fields)
#*                                       *#$f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                          #end for
#*                                       *#);
    struct packet p = ${className}ToPacket(&m);
    sendPacket(p);
}
#end for 
#end for 

// ================
// Message Handlers
// ================

void handlePacket(struct packet p)
{
    uint32_t cid;
    load_uint32(p, 0, &cid);

    switch (cid) {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
        {
            DEBUG("received $className\n");
	        struct ${className} m = ${className}FromPacket(p);
            handle${className}(&m);
            break;
        }
        #end for 
        #end for
        default:
            DEBUG("received unknown packet\n");
    }
}

// Implementations of handleXMessage are separate
