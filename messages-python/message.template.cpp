/***  This is a generated file, do not edit ***/
\#include "${headerFile}"
\#include <boost/serialization/vector.hpp>
\#include <boost/serialization/map.hpp>
\#include <boost/make_shared.hpp>
\#include <boost/thread.hpp>
\#include <boost/noncopyable.hpp>

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

template<typename T>
struct BufferingThread: public BufferingThreadBase, boost::noncopyable
{
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
            
            if(m_die){
                break;
            }

            if(m_buffer){
                T temp = boost::dynamic_pointer_cast<typename T::value_type>(m_buffer);
                if(temp){
                    m_buffer.reset();
                    l.unlock();
                    (m_obs.*m_notify)(temp);
                }else{
                    error() << "incorrect message type for this buffer!";
                }
            }else{
                error() << "buffering thread wrongly notified!";
            }

        }
    }

    void (BufferedMessageObserver::*m_notify)(T);
};

BufferedMessageObserver::BufferedMessageObserver()
    : m_maps_mtx(boost::make_shared<boost::shared_mutex>())
{
}

BufferedMessageObserver::~BufferedMessageObserver()
{
    boost::unique_lock<boost::shared_mutex> l(*m_maps_mtx);
    foreach(msgtype_btthread_map_t::value_type& v, m_threads)
        if(v.second){
            v.second->m_die = true;
            v.second->m_condition->notify_one();
            v.second.reset();
            assert(m_boost_threads[v.first]);
            m_boost_threads[v.first]->join();
            m_boost_threads[v.first].reset();
            info() << "Double-Buffering disabled for" << v.first << "messages";
        }
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void BufferedMessageObserver::on${className}($classPtr m)
{
    boost::shared_lock<boost::shared_mutex> l(*m_maps_mtx);
    msgtype_btthread_map_t::iterator bt = m_threads.find(MessageType::$m.name);
    if(bt != m_threads.end() && bt->second)
    {
        boost::unique_lock<boost::mutex> l(*(bt->second->m_mutex));
        bt->second->m_buffer = m;
        l.unlock();
        bt->second->m_condition->notify_one();
    }
    else
    {
        on${className}Buffered(m);
    }

}
#end for
#end for

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void BufferedMessageObserver::on${className}Buffered($classPtr) { }
#end for
#end for

void BufferedMessageObserver::setDoubleBuffered(MessageType::e mt, bool v)
{
    using boost::thread;
    using boost::make_shared;
    using boost::ref;
    using boost::shared_ptr;

    boost::unique_lock<boost::shared_mutex> l(*m_maps_mtx);
    if(v && !m_threads[mt])
    {
        assert(!m_boost_threads[mt]);
        
        thread_ptr_t boost_thread;

        switch(mt)
        {
            #for $g in $groups
            #for $m in $g.messages
            #set $className = $m.name + "Message"
            #set $classPtr = $className + "_ptr"
            case MessageType::$m.name:
            {
                typedef BufferingThread<$classPtr> thread_t;
                shared_ptr<thread_t> t = make_shared<thread_t>(
                    ref<this_t>(*this), &this_t::on${className}Buffered
                );
                m_boost_threads[mt] = make_shared<thread>(ref<thread_t>(*t));
                m_threads[mt] = t;
                break;
            }
            #end for
            #end for
            default:
                break;
        }
        
        if(m_threads[mt])
        {
            info() << "Double-Buffering enabled for" << mt << "messages";
        }
        else
        {
            error() << "Invalid message type:" << mt;
        }
    }
    else
    {
        m_threads[mt]->m_die = true;
        m_threads[mt]->m_condition->notify_one();
        m_threads[mt].reset();
        m_boost_threads[mt]->join();
        m_boost_threads[mt].reset();

        info() << "Double-Buffering disabled for" << mt << "messages";        
    }
}


DynamicObserver::DynamicObserver(){ }
DynamicObserver::~DynamicObserver(){ }
        
void DynamicObserver::setCallback(MessageType::e id, callback_f_ptr f)
{
    m_callbacks[id] = f;
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $ptrName = $className + "_ptr"
void DynamicObserver::on${className}Buffered($ptrName m)
{
    callback_map_t::iterator i = m_callbacks.find(MessageType::$m.name);
    if(i != m_callbacks.end() && i->second)
        (*i->second)(m);
}
#end for
#end for



DebugMessageObserver::DebugMessageObserver(unsigned int level) : m_level(level)
{
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void DebugMessageObserver::on${className}($classPtr m)
{
    debug(m_level) << "DebugMessageObserver: " << *m;
}
#end for
#end for



UnknownMessageIdException::UnknownMessageIdException(uint32_t id) : m_id(id)
{
}
const char * UnknownMessageIdException::what() const throw()
{
    std::string message = MakeString() << "Unknown message id: " << m_id;
    return message.c_str();
}


MessageSource::MessageSource()
{
}
void MessageSource::notifyObservers(boost::shared_ptr<const byte_vec_t> bytes)
{
    if (bytes->size() < 4)
        throw std::out_of_range("Buffer too small to contain message id");

    int id = *reinterpret_cast<const uint32_t*>(bytes->data());
    switch (id)
    {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
        {
            boost::shared_ptr<$className> m = $className::fromBytes(bytes);
            foreach(observer_ptr_t o, m_observers)
            {
                o->on${className}(m);
            }
            break;
        }
        #end for
        #end for
        default:
            throw UnknownMessageIdException(id);
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

