/***  This is a generated file, do not edit ***/
\#ifndef __MESSAGE_MESSAGES_H__
\#define __MESSAGE_MESSAGES_H__

\#include "messages_fwd.h"

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
\#ifndef foreach
\#    include <boost/foreach.hpp>
\#    define foreach BOOST_FOREACH
\#endif

\#include <common/image.h>

// Message data type definitions
typedef std::string byte_vec_t;


// ================
// Type definitions
// ================

#for $s in $structs
struct $s.name
{
    #for $f in $s.fields
    $toCPPType($f.type) $f.name;
    #end for
};
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

        virtual boost::shared_ptr<const byte_vec_t> toBytes() const = 0;

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
#*                  *#$toCPPType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                     #end for
#*                  *#);
        #end if

        #for $f in $m.fields
        const $toCPPType($f.type)& ${f.name}() const;
        void ${f.name}($toCPPType($f.type) const& $f.name);

        // interface for python export: no overloads:
        const $toCPPType($f.type)& get_${f.name}() const{ return ${f.name}(); }
        void set_${f.name}($toCPPType($f.type) const& ${f.name}_value){ ${f.name}(${f.name}_value); }
        
        #end for

        static boost::shared_ptr<${className}> fromBytes(boost::shared_ptr<const byte_vec_t> bytes);
        virtual boost::shared_ptr<const byte_vec_t> toBytes() const;
        
        template<class Archive>
        void save(Archive & ar, const unsigned int /*version*/) const
        {
            ar << m_id;

            #for $f in $m.fields
            ar << m_$f.name;
            #end for
        }
        template<class Archive>
        void load(Archive & ar, const unsigned int /*version*/) const
        {
            uint32_t buf_id;
            ar >> buf_id;
            if (buf_id != m_id)
            {
                throw std::invalid_argument("Attempted to create $className with invalid id");
            }
            #for $f in $m.fields
            ar >> m_$f.name;
            #end for
        }
        BOOST_SERIALIZATION_SPLIT_MEMBER()

    private:
        #for i, f in $enumerate($m.fields)
        mutable $toCPPType($f.type) m_$f.name;
        #end for
        
        mutable boost::shared_ptr<const byte_vec_t> m_bytes;
        void deserialize() const;

    template<typename char_T, typename traits>
    friend std::basic_ostream<char_T, traits>& operator<<(
        std::basic_ostream<char_T, traits>& os, $className const& m);
};
BOOST_CLASS_IMPLEMENTATION($className, boost::serialization::object_serializable)
BOOST_CLASS_IS_WRAPPER($className)

#end for
#end for


\#endif//__MESSAGE_MESSAGES_H__
