/***  This is a generated file, do not edit ***/
\#include "serialmess.h"

\#include "messages.h"

\#include <utility/serialisation.h>

#for $e in $enums
void cauv::serialise(svec_ptr p, $e.name::e const& e){
    serialise(p, $toCPPType($e.type)(e));
}

int32_t cauv::deserialise(const_svec_ptr p, int32_t i, $e.name::e& e){
    int32_t r;
    $toCPPType($e.type) t = 0;
    r = deserialise(p, i, t);
    e = $e.name::e(t);
    return r;
}

#end for


#for $s in $structs
void cauv::serialise(svec_ptr p, $s.name const& v){
    #for f in $s.fields
    serialise(p, v.$f.name);
    #end for
}

int32_t cauv::deserialise(const_svec_ptr p, int32_t i, $s.name& v){
    int32_t b = i;
    #for f in $s.fields
    b += deserialise(p, b, v.$f.name);
    #end for
    return b - i;
}

#end for

#for $v in $variants
void cauv::serialise(svec_ptr p, $v.name const& v){
    serialise(p, uint32_t(v.which()));
    switch(v.which()){
        default:
        #for $i, $t in $enumerate($v.types)
        case $i:
            serialise(p, boost::get<$toCPPType($t)>(v));
            break;
        #end for
    }
}

int32_t cauv::deserialise(const_svec_ptr p, int32_t i, $v.name& v){
    int32_t b = i;
    uint32_t discriminant;
    b += deserialise(p, b, discriminant);
    switch(discriminant){
        default:
        #for $i, $t in $enumerate($v.types)
        case $i:
        {
            $toCPPType($t) t;
            b += deserialise(p, b, t);
            v = t;
            break;
        }
        #end for
    }
    return b - i;
}
#end for

template<typename T>
static void serialiseLazyField(cauv::svec_ptr p, T const& v){
    uint32_t initial_size = p->size();
    cauv::serialise(p, uint32_t(0));
    cauv::serialise(p, v);
    *reinterpret_cast<uint32_t*>(&(*p)[initial_size]) = p->size() - initial_size;
}

/****   Message Serialisation   ****/ 
#for $g in $groups

#for $m in $g.messages
#set $className = $m.name + "Message"
#if len($m.fields)
void cauv::serialise(svec_ptr p, $className const& v){
    ## reserve estimated minimum message size
    p->reserve(#echo 4 *(1 + len(m.fields))#);
    ## message type
    cauv::serialise(p, uint32_t($m.id)); 
    #for f in $m.fields
    #if $f.lazy
    serialiseLazyField(p, v.m_$f.name);
    #else
    cauv::serialise(p, v.m_$f.name);
    #end if
    #end for
}
#else
void cauv::serialise(svec_ptr p, $className const&){
    cauv::serialise(p, uint32_t($m.id)); 
}
#end if
#end for
#end for

