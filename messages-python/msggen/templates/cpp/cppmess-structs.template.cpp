/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#include <debug/cauv_debug.h>

#for $s in $structs
\#include "${s.name}.h"
#end for

using namespace cauv;

#for $s in $structs
#if s.include is None
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

#if $s.numEqualityFields > 0
bool cauv::${s.name}::operator==(cauv::$s.name const& other) const
{
    return
    #for i, f in $enumerate($s.fields)
        #if f.equality
        $f.name == other.$f.name#if $i < $s.numEqualityFields - 1# &&#else#;#end if
        #end if
    #end for
}
#end if

#if $s.numCompareFields > 0
bool cauv::${s.name}::operator<(cauv::$s.name const& other) const
{
    #for i, f in $enumerate($s.fields)
    #if f.compare
    if($f.name < other.$f.name)
        return true;
    else if(!($f.name == other.$f.name))
        return false;
    #end if
    #end for
    return false;
}
#end if
#end if

#end for 
