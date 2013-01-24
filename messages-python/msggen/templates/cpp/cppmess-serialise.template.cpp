/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#include "serialise.h"
#for $s in $structs
\#include "${s.name}.h"
#end for
#for $e in $enums
\#include "${e.name}.h"
#end for
#for $v in $variants
\#include "${v.name}.h"
#end for
#for $t in $included_types
\#include $t.location
#end for
\#include <utility/serialisation.h>
\#include <utility/string.h>

#for $e in $enums
void cauv::serialise(svec_ptr p, $e.name::e const& e){
    serialise(p, $toCPPType($e.type)(e));
}

int32_t cauv::deserialise(const_svec_ptr p, uint32_t i, $e.name::e& e){
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

int32_t cauv::deserialise(const_svec_ptr p, uint32_t i, $s.name& v){
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
            serialise(p, boost::get< $toCPPType($t) >(v));
            break;
        #end for
    }
}

int32_t cauv::deserialise(const_svec_ptr p, uint32_t i, $v.name& v){
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


#for $e in $enums
std::string cauv::chil($e.name::e const& e){
    return mkStr() << int32_t(e);
}
#end for


#for $s in $structs
std::string cauv::chil($s.name const& v){
    mkStr r;
    r << "(";
    #for i,f in $enumerate($s.fields)
    #if i+1 != len($s.fields)
    r << chil(v.${f.name}) << ",";
    #else
    r << chil(v.${f.name});
    #end if
    #end for
    return r << ")";
}
#end for

#for $v in $variants
std::string cauv::chil($v.name const& v){
    mkStr r;
    r << "(" << v.which() << ",";
    switch(v.which()){
        default:
        #for $i, $t in $enumerate($v.types)
        case $i:
            r << chil(boost::get< $toCPPType($t) >(v));
            break;
        #end for
    }
    return r << ")";
}
#end for

// Message serialisation is defined in the message cpp files
