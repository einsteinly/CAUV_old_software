/***  This is a generated file, do not edit ***/
\#include <common/cauv_utils.h>
\#include <debug/cauv_debug.h>

#for $s in $structs
\#include "${s.name}.h"
#end for

using namespace cauv;

#for $s in $structs
cauv::${s.name}::${s.name}() { }
#if $len($s.fields) > 0
cauv::${s.name}::${s.name}(#slurp
                     #for i, f in $enumerate($s.fields)
#*                  *#$toCPPType($f.type) const& $f.name#if $i < $len($s.fields) - 1#, #end if##slurp
                     #end for
#*                  *#) : #slurp
                          #for i, f in $enumerate($s.fields)
#*                       *#${f.name}($f.name)#if $i < $len($s.fields) - 1#, #end if##slurp
                          #end for
{ }
#end if

bool cauv::${s.name}::operator==(cauv::$s.name const& other) const
{
    return
    #for i, f in $enumerate($s.fields)
        $f.name == other.$f.name#if $i < $len($s.fields) - 1# &&#else#;#end if
    #end for
}

#end for 
