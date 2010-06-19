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

\#include <common/streamops.h>
\#include <common/image.h>

// Message data type definitions
typedef std::string byte_vec_t;
typedef std::ostringstream byte_ostream_t;
typedef std::istringstream byte_istream_t;


// ================
// Type definitions
// ================

#for $t in $unknown_types
class $t;
#end for

#for $e in $enums
namespace $e.name
{
    enum e
    {
        #for $i, $v in $enumerate($e.values)
        $v.name = $v.value,
        #end for
        NumValues = $len($e.values)
    };
} // namespace $e.name
#end for

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

namespace MessageType
{
    enum e
    {
        #set $num_values = 0
        #for $g in $groups
        #for $m in $g.messages
        $m.name = $m.id,
        #set $num_values += 1
        #end for
        #end for
        NumValues = $num_values
    };
} // namespace $e.name

// Base message class
class Message
{
    public:
        virtual ~Message();

        std::string group() const;
        uint32_t id() const;

        virtual boost::shared_ptr<const byte_vec_t> toBytes() const = 0;

    protected:
        uint32_t m_id;
        std::string m_group;

        Message(uint32_t id, std::string group);

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



// =======================
// Message Source/Observer
// =======================

class MessageObserver
{
    public:
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        typedef boost::shared_ptr<const $className> $ptrName;
        #end for
        #end for

        virtual ~MessageObserver();
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}($ptrName m);
        #end for
        #end for
    
    // Ideally protected, but boost.python pointer_holder requires puplic
    // default constructor to be available in order to allow pointers to this
    // type
    //protected:
        MessageObserver();
};

class BufferingThreadBase;
class BufferedMessageObserver: public MessageObserver
{
    typedef BufferedMessageObserver this_t;
    typedef boost::shared_ptr<boost::thread> thread_ptr_t;
    typedef boost::shared_ptr<BufferingThreadBase> bthread_ptr_t;

    public:
        virtual ~BufferedMessageObserver();

        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}($ptrName m);
        #end for
        #end for

        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}Buffered($ptrName m);
        #end for
        #end for

        void setDoubleBuffered(MessageType::e, bool);

    // Ideally protected, but boost.python pointer_holder requires puplic
    // default constructor to be available in order to allow pointers to this
    // type
    //protected:
        BufferedMessageObserver();

    private:
        std::map<MessageType::e, thread_ptr_t> m_boost_threads;
        std::map<MessageType::e, bthread_ptr_t> m_threads;
};

class DebugMessageObserver: public MessageObserver
{
    public:
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}($ptrName m);
        #end for
        #end for
};
class MessageSource
{
    public:
        void notifyObservers(boost::shared_ptr<const byte_vec_t> bytes);
        void addObserver(boost::shared_ptr<MessageObserver> o);
        void removeObserver(boost::shared_ptr<MessageObserver> o);
        void clearObservers();

    
    // Ideally protected, but boost.python pointer_holder requires puplic
    // default constructor to be available in order to allow pointers to this
    // type
    // protected:
        MessageSource();

    protected:
        std::list< boost::shared_ptr<MessageObserver> > m_obs;
};



// ===========================
// Struct & Enum Serialization
// ===========================

namespace boost {
namespace serialization {

#for s in $structs
template<class Archive>
void serialize(Archive & ar, $s.name& val, const unsigned int /*version*/)
{
    #for f in $s.fields
    ar & val.$f.name;
    #end for
}
#end for

} // namespace serialization
} // namespace boost


// Enum serialization is pretty hacky for now, since boost::serialization
// blindly converts all enums to ints for some reason

namespace boost {
namespace archive {
namespace detail {

#for e in $enums
template<> template <>
void save_enum_type<binary_oarchive>::invoke<$e.name::e>(binary_oarchive &ar, const $e.name::e &val);
template<> template <>
void load_enum_type<binary_iarchive>::invoke<$e.name::e>(binary_iarchive &ar, $e.name::e &val);

#end for

} // namespace detail
} // namespace archive
} // namespace boost



// =============
// Printing Code
// =============

#for $s in $structs
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $s.name const& s)
{
    os << "$s.name {";
    #for i, f in $enumerate($s.fields)
    os << " $f.name = " << s.$f.name#if $i < $len($s.fields) - 1#<< ","#end if#;
    #end for
    os << " }";
    return os;
}
#end for

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, MessageType::e const& e)
{
    switch(e)
    {
        #for $g in $groups
        #for $m in $g.messages
        case MessageType::$m.name:
            return os << "MessageType::$m.name";
        #end for
        #end for
        default:
            return os << "MessageType::Unknown=" << int(e);
    }
}


#for $e in $enums
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $e.name::e const& e)
{
    switch(e)
    {
        #for $v in $e.values
        case $e.name::$v.name:
            return os << "$e.name::$v.name";
        #end for
        default:
            return os << "$e.name::Unknown=" << int(e);
    }
}
#end for


template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Message const& m)
{
    os << "Message {";
    os << " id = " << m.m_id << ",";
    os << " group = " << m.m_group;
    os << " }";
    return os;
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $className const& #if $len($m.fields)#m#end if#)
{
    #if $len($m.fields)#m.deserialize();#end if#
    os << "$className {";
    #for i, f in $enumerate($m.fields)
    #if $f.name == "int8" or $f.name == "byte" 
    os << " $f.name = " << (int)m.m_$f.name#if $i < $len($m.fields) - 1# << ","#end if#;
    #else
    os << " $f.name = " << m.m_$f.name#if $i < $len($m.fields) - 1# << ","#end if#;
    #end if
    #end for
    os << " }";
    return os;
}

#end for
#end for

\#endif//__MESSAGES_H__
