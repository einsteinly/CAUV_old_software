\#include "${headerFile}"

// ===============
// Message Structs
// ===============
void destroyPacket(struct packet* m)
{
    free(m->data);
}

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
struct $className* empty${className}()
{
    struct $className* m = malloc(sizeof(struct $className));
    m->id = $m.id;
    return m;
}
struct $className* new${className}(#slurp
                                   #for i, f in $enumerate($m.fields)
#*                                *#$toCType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                   #end for
#*                                *#)
{
    struct $className* m = empty${className}();
    #for $f in $m.fields
    m->$f.name = $f.name;
    #end for
}
struct ${className}* ${className}FromPacket(struct packet p)
{
    struct $className* m = empty${className}();
    int pos = 4; // Skip the message id (not as safe, but faster)

    #for $f in $m.fields
    pos = load_$loadsavesuffix($f.type)(p, pos, &m->$f.name);
    #end for

    return m;
}
struct packet ${className}ToPacket(struct $className* m)
{
    struct packet p;
    int pos = 0;

    pos = save_uint32(p, pos, &m->id);

    #for $f in $m.fields
    pos = save_$loadsavesuffix($f.type)(p, pos, &m->$f.name);
    #end for

    return p;
}

#end for 
#end for 


// =======================
// Message Sending Helpers
// =======================

void sendMessage(struct Message* m)
{
    struct packet p = toPacket(m);
    sendPacket(p);
    destroyPacket(&p);
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
    struct $className* m = new${className}(#slurp
                                           #for i, f in $enumerate($m.fields)
#*                                        *#$f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                                           #end for
#*                                        *#);
    struct packet p = ${className}ToPacket(m);
    sendPacket(p);
    destroyPacket(&p);
    free(m);
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
	        struct ${className}* m = ${className}FromPacket(p);
            handle${className}(m);
            free(m);
            break;
        }
        #end for 
        #end for
        default:
            DEBUG("received unknown packet\n");
    }
}

// Implementations of handleXMessage are separate
