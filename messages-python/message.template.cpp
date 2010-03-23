/***  This is a generated file, do not edit ***/
\#include "messages.h"
\#include <boost/serialization/vector.hpp>
\#include <boost/serialization/map.hpp>
\#include <boost/make_shared.hpp>

// ===============
// Message Classes
// ===============

// Base message class
Message::Message(uint32_t id, std::string group) : m_id(id), m_group(group)
{
}

Message::~Message()
{
}

std::string Message::group() const
{
    return m_group;
}

uint32_t Message::id() const
{
    return m_id;
}

#for $g in $groups
// $g.name group

#for $m in $g.messages
#set $className = $m.name + "Message"
#if $len($m.fields) > 0
${className}::${className}()
    : Message($m.id, "$g.name"),
      #for i, f in $enumerate($m.fields)
      m_${f.name}(),
      #end for
      m_bytes()
{
}
${className}::${className}(#slurp
                           #for i, f in $enumerate($m.fields)
#*                        *#$toCPPType($f.type) $f.name#if $i < $len($m.fields) - 1#, #end if##slurp
                           #end for
#*                        *#)
    : Message($m.id, "$g.name"),
      #for i, f in $enumerate($m.fields)
      m_${f.name}($f.name),
      #end for
      m_bytes()
{
}
#else
${className}::${className}()
    : Message($m.id, "$g.name"),
      m_bytes()
{
}
#end if

#for f in $m.fields
const $toCPPType($f.type)& $className::${f.name}() const
{
    deserialize();
    return m_$f.name;
}
void $className::${f.name}($toCPPType($f.type) const& $f.name)
{
    deserialize();
    m_$f.name = $f.name;
}

#end for

void $className::deserialize() const
{
    if (m_bytes)
    {
        byte_istream_t iss(*m_bytes, std::ios_base::in|std::ios_base::binary);
        boost::archive::binary_iarchive ar(iss, boost::archive::no_header);
        ar & *this;
        m_bytes.reset();
    }
}

boost::shared_ptr<$className> $className::fromBytes(boost::shared_ptr<const byte_vec_t> bytes)
{
    boost::shared_ptr<$className> ret = boost::make_shared<$className>();
    ret->m_bytes = bytes;
    return ret;
}
boost::shared_ptr<const byte_vec_t> $className::toBytes() const
{
    if (m_bytes)
    {
        return m_bytes;
    }
    else
    {
        byte_ostream_t oss(std::ios_base::out|std::ios_base::binary);
        boost::archive::binary_oarchive ar(oss, boost::archive::no_header);
        ar & *this;
        return boost::make_shared<const byte_vec_t>(oss.str());
    }
}


#end for
#end for



// =======================
// Message Source/Observer
// =======================

MessageObserver::MessageObserver()
{
}
MessageObserver::~MessageObserver()
{
}
#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void MessageObserver::on${className}(boost::shared_ptr<const $className>) {}
#end for
#end for 

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
void DebugMessageObserver::on${className}(boost::shared_ptr<const $className> m)
{
    info() << "DebugMessageObserver: " << *m;
}
#end for
#end for


MessageSource::MessageSource()
{
}
void MessageSource::addObserver(boost::shared_ptr<MessageObserver> o)
{
    m_obs.push_back(o);
}
void MessageSource::removeObserver(boost::shared_ptr<MessageObserver> o)
{
    m_obs.remove(o);
}
void MessageSource::clearObservers()
{
    m_obs.clear();
}
void MessageSource::notifyObservers(boost::shared_ptr<const byte_vec_t> bytes)
{
    if (bytes->size() < 4)
        throw std::out_of_range("Buffer too small to contain message id");

    switch(*reinterpret_cast<const uint32_t*>(bytes->data()))
    {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
        {
            boost::shared_ptr<$className> m = $className::fromBytes(bytes);
            foreach(boost::shared_ptr<MessageObserver> o, m_obs)
            {
                o->on${className}(m);
            }
            break;
        }
        #end for
        #end for
        default:
            throw(std::out_of_range("Unknown message id"));
    }
}



// ==================
// Enum Serialization
// ==================

// Enum serialization is pretty hacky for now, since boost::serialization
// blindly converts all enums to ints for some reason

namespace boost {
namespace archive {
namespace detail {

#for e in $enums
template<> template <>
void save_enum_type<binary_oarchive>::invoke<$e.name::e>(binary_oarchive &ar, const $e.name::e &val)
{
    $toCPPType($e.type) typedVal = static_cast<const $toCPPType($e.type)>(val);
    ar << typedVal;
}
template<> template <>
void load_enum_type<binary_iarchive>::invoke<$e.name::e>(binary_iarchive &ar, $e.name::e &val)
{
    $toCPPType($e.type) typedVal;
    ar >> typedVal;
    val = static_cast<$e.name::e>(typedVal);
}

#end for

} // namespace detail
} // namespace archive
} // namespace boost
