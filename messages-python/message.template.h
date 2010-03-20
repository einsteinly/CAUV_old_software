/***  This is a generated file, do not edit ***/
\#ifndef __MESSAGES_H__
\#define __MESSAGES_H__

\#include <string>
\#include <sstream>
\#include <stdexcept>
\#include <vector>
\#include <list>
\#include <map>
\#include <boost/cstdint.hpp>
\#include <boost/shared_ptr.hpp>
\#include <boost/archive/binary_oarchive.hpp>
\#include <boost/archive/binary_iarchive.hpp>
\#include <boost/archive/detail/oserializer.hpp>
\#include <boost/archive/detail/iserializer.hpp>
\#ifndef foreach
\#    include <boost/foreach.hpp>
\#    define foreach BOOST_FOREACH
\#endif

\#include <common/vector_streamops.h>
\#include <common/image.h>

// message data type definitions
typedef std::string byte_vec_t;
typedef std::ostringstream byte_ostream_t;
typedef std::istringstream byte_istream_t;


// Type definitions
//
#for t in $unknown_types
class $t;
#end for

#for s in $structs
struct $s.name
{
    #for f in $s.fields
    $toCPPType($f.type) $f.name;
    #end for
}
#end for

#for e in $enums
enum $e.name
{
    #for i,v in $ienumerate($e.values)
    $v.name = $v.value#if $i < $e.values.len() - 1 then echo "," else echo ""# 
    #end for
}
#end for





// Printing Code

#for s in $structs
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $s.name const& s)
{
    os << "$s.name {" << endl;
    #for f in $s.fields
    os << " $f.name = " << s.$f.name << endl;
    #end for
    os << '}';
    return os;
}
#end for
