/***  This is a generated file, do not edit ***/
\#include "message_observers.h"

\#include <boost/make_shared.hpp>
\#include <boost/thread/thread.hpp>
\#include <boost/thread/mutex.hpp>
\#include <boost/thread/shared_mutex.hpp>
\#include <boost/thread/condition_variable.hpp>
\#include <boost/noncopyable.hpp>

\#include <utility/string.h>
\#include <utility/serialisation.h>
\#include <common/cauv_utils.h>
\#include <debug/cauv_debug.h>

\#include "types/message.h"
#for $g in $groups
#for $m in $g.messages
\#include "types/${m.name}Message.h"
#end for
#end for

using namespace cauv;

cauv::MessageObserver::MessageObserver() { }
cauv::MessageObserver::~MessageObserver() { }
#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void cauv::MessageObserver::on${className}($classPtr) { }
#end for
#end for 

namespace cauv{

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

    void (cauv::BufferedMessageObserver::*m_notify)(T);
};

} // namespace cauv

cauv::BufferedMessageObserver::BufferedMessageObserver()
    : m_maps_mtx(boost::make_shared<boost::shared_mutex>())
{
}

cauv::BufferedMessageObserver::~BufferedMessageObserver()
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
void cauv::BufferedMessageObserver::on${className}($classPtr m)
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
void cauv::BufferedMessageObserver::on${className}Buffered($classPtr) { }
#end for
#end for

void cauv::BufferedMessageObserver::setDoubleBuffered(MessageType::e mt, bool v)
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
                typedef cauv::BufferingThread<$classPtr> thread_t;
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


cauv::DynamicObserver::DynamicObserver(){ }
cauv::DynamicObserver::~DynamicObserver(){ }
        
void cauv::DynamicObserver::setCallback(MessageType::e id, callback_f_ptr f)
{
    m_callbacks[id] = f;
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $ptrName = $className + "_ptr"
void cauv::DynamicObserver::on${className}Buffered($ptrName m)
{
    callback_map_t::iterator i = m_callbacks.find(MessageType::$m.name);
    if(i != m_callbacks.end() && i->second)
        (*i->second)(m);
}
#end for
#end for



cauv::DebugMessageObserver::DebugMessageObserver(unsigned int level) : m_level(level)
{
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
#set $classPtr = $className + "_ptr"
void cauv::DebugMessageObserver::on${className}($classPtr m)
{
    debug(m_level) << "DebugMessageObserver: " << *m;
}
#end for
#end for



cauv::UnknownMessageIdException::UnknownMessageIdException(uint32_t id) : m_id(id)
{
}
const char * cauv::UnknownMessageIdException::what() const throw()
{
    std::string message = MakeString() << "Unknown message id: " << m_id;
    return message.c_str();
}


cauv::MessageSource::MessageSource()
{
}
void cauv::MessageSource::notifyObservers(const_svec_ptr bytes)
{
    if (bytes->size() < 4)
        throw std::out_of_range("Buffer too small to contain message id");

    int id = *reinterpret_cast<const uint32_t*>(&bytes->front());
    switch (id)
    {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
        {
            boost::shared_ptr<$className> m = $className::fromBytes(bytes);
            \#ifdef CAUV_DEBUG_MESSAGES
            debug(12) << "MessageSource::notifyObservers: " << m;
            \#endif
            foreach(observer_ptr_t o, m_observers)
                o->on${className}(m);
            break;
        }
        #end for
        #end for
        default:
            throw UnknownMessageIdException(id);
    }
}

