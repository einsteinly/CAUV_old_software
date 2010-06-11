/***  This is a generated file, do not edit ***/
\#include "${headerFile}"
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
#set $classPtr = $className + "_ptr"
void MessageObserver::on${className}($classPtr) {}
#end for
#end for 

BufferedMessageObserver::BufferedMessageObserver()
{
}

BufferedMessageObserver::~BufferedMessageObserver()
{
    // TODO: MUST stop all threads before this is destroyed!
}

struct BufferingThreadBase
{ 
    BufferingThreadBase(BufferedMessageObserver& obs)
        : m_die(false), m_buffer(),
          m_condition(boost::make_shared<boost::condition_variable>()),
          m_mutex(boost::make_shared<boost::mutex>()),
          m_obs(obs)
    {
    }

    bool m_die;
    boost::shared_ptr<const Message> m_buffer;
    boost::shared_ptr<boost::condition_variable> m_condition;
    boost::shared_ptr<boost::mutex> m_mutex;
    BufferedMessageObserver& m_obs;
};

template<unsigned N_id, typename T>
struct BufferingThread: public BufferingThreadBase{
    BufferingThread(BufferedMessageObserver& obs, void (BufferedMessageObserver::*f)(T))
        : BufferingThreadBase(obs), m_notify(f)
    {
    }

    void operator()()
    {
        for(;;)
        {
            boost::unique_lock<boost::mutex> l(*m_mutex);
            if(!m_buffer)
                m_condition->wait(l);
            if(m_die)
                break;
            T temp = boost::dynamic_pointer_cast<typename T::value_type>(m_buffer);
            l.unlock();
            (m_obs.*m_notify)(temp);
        }
    }

    void (BufferedMessageObserver::*m_notify)(T);
};

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void BufferedMessageObserver::on${className}($classPtr m)
{
    boost::shared_ptr<BufferingThreadBase> bt = m_threads[MessageType::$m.name];
    if(bt)
    {
        boost::unique_lock<boost::mutex> l(*(bt->m_mutex));
        bt->m_buffer = m;
        l.unlock();
        bt->m_condition->notify_one();
    }
}
#end for
#end for

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void BufferedMessageObserver::on${className}Buffered($classPtr) {}
#end for
#end for

void BufferedMessageObserver::setDoubleBuffered(MessageType::e mt, bool v)
{
    if(v && !m_threads[mt])
    {
        switch(mt)
        {
            #for $g in $groups
            #for $m in $g.messages
            #set $className = $m.name + "Message"
            #set $classPtr = $className + "_ptr"
            case MessageType::$m.name:
            {
                boost::shared_ptr<BufferingThread<MessageType::$m.name, $classPtr> > t =
                    boost::make_shared<BufferingThread<MessageType::$m.name, $classPtr> >(
                        boost::ref<BufferedMessageObserver>(*this), &BufferedMessageObserver::on${className}
                );

                m_boost_threads[MessageType::$m.name] = boost::make_shared<boost::thread>(*t);
                m_threads[MessageType::$m.name] = t;

                break;
            }   
            #end for
            #end for
        }
    }
    else
    {
        m_threads[MessageType::$m.name]->m_die = true;
        m_threads[MessageType::$m.name]->m_condition->notify_one();
        m_threads[MessageType::$m.name].reset();
        m_boost_threads[MessageType::$m.name]->join();
        m_boost_threads[MessageType::$m.name].reset();
    }
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void DebugMessageObserver::on${className}($classPtr m)
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
            throw std::out_of_range("Unknown message id");
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
