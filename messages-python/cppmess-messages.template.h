/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_MESSAGE_MESSAGES_H__
\#define __CAUV_MESSAGE_MESSAGES_H__

\#include "messages_fwd.h"

\#include <string>
\#include <vector>
\#include <list>
\#include <map>
\#include <boost/cstdint.hpp>
\#include <boost/shared_ptr.hpp>
\#include <boost/variant.hpp>
\#ifndef foreach
\#    include <boost/foreach.hpp>
\#    define foreach BOOST_FOREACH
\#endif

\#include <common/image.h>
\#include <utility/serialisation-types.h>


namespace cauv{

// ================
// Type definitions
// ================

#for $s in $structs
struct $s.name
{
    #for $f in $s.fields
    $toCPPType($f.type) $f.name;
    #end for
        
    ${s.name}();
    #if len($s.fields) > 0
    ${s.name}(#slurp
              #for i, f in $enumerate($s.fields)
#*           *#$toCPPType($f.type) const& $f.name#if $i < $len($s.fields) - 1#, #end if##slurp
              #end for
#*           *#);
    #end if 
};

#end for

#for $v in $variants
typedef boost::variant<
    #for $i, $t in $enumerate($v.types)
    #if $i < $len($v.types) -1
        $toCPPType($t),
    #else
        $toCPPType($t)
    #end if
    #end for
> $v.name;

#end for

// ===============
// Message Classes
// ===============

// Base message class
class Message
{
    public:
        virtual ~Message();

        std::string const& group() const;
        uint32_t id() const;

        virtual const_svec_ptr toBytes() const = 0;

    protected:
        uint32_t m_id;
        std::string m_group;

        Message(uint32_t id, std::string const& group);

    template<typename char_T, typename traits>
    friend std::basic_ostream<char_T, traits>& operator<<(
        std::basic_ostream<char_T, traits>& os, Message const& m);
};

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
class $className : public Message
{
    public:
        ${className}();
        #if $len($m.fields) > 0
        ${className}(#slurp
                     #for i, f in $enumerate($m.fields)
#*                  *#$toCPPType($f.type) const& $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                     #end for
#*                  *#);
        #end if

        #for $f in $m.fields
        const $toCPPType($f.type)& ${f.name}() const;
        const $toCPPType($f.type)& get_${f.name}() const{ return ${f.name}(); }
        void ${f.name}($toCPPType($f.type) const& $f.name);
        void set_${f.name}($toCPPType($f.type) const& ${f.name}_value){ ${f.name}(${f.name}_value); }
        
        #end for

        static boost::shared_ptr<${className}> fromBytes(const_svec_ptr bytes);
        virtual const_svec_ptr toBytes() const;

    private:
#if $len($m.fields) > 0
        void deserialise() const;
        inline void checkDeserialised() const{
            if(!m_deserialised){
                deserialise();
                m_deserialised = true;
            }
        }
        mutable bool m_deserialised;

    #for i, f in $enumerate($m.fields)
        mutable $toCPPType($f.type) m_$f.name;
    #end for

    #if $m.numLazyFields() > 0
        mutable std::set<int> m_lazy_fields_deserialised;
        #for i, f in $enumerate($m.fields)
            #if $f.lazy
        mutable uint32_t m_lazy_field_${i}_offset;
            #end if
        #end for
    #end if
        
        mutable const_svec_ptr m_bytes;
#end if

    template<typename char_T, typename traits>
    friend std::basic_ostream<char_T, traits>& operator<<(
        std::basic_ostream<char_T, traits>& os, $className const& m);
    friend void serialise(svec_ptr, $className const&);
};

#end for
#end for

} // namespace cauv

\#endif // __CAUV_MESSAGE_MESSAGES_H__

