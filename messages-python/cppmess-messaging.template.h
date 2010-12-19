/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_MESSAGES_H__
\#define __CAUV_MESSAGES_H__

\#include "messages_messages.h"

\#include <utility/observable.h>
\#include <utility/streamops.h>
\#include <common/image.h>

// Forward Declarations outside cauv namespace:
namespace boost{
class thread;
class shared_mutex;
} // namespace boost

namespace cauv{

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

struct BufferingThreadBase;
class BufferedMessageObserver: public MessageObserver
{
    typedef BufferedMessageObserver this_t;
    typedef boost::shared_ptr<boost::thread> thread_ptr_t;
    typedef boost::shared_ptr<BufferingThreadBase> btthread_ptr_t;
    typedef std::map<MessageType::e, thread_ptr_t> msgtype_thread_map_t;
    typedef std::map<MessageType::e, btthread_ptr_t> msgtype_btthread_map_t;

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
        boost::shared_ptr<boost::shared_mutex> m_maps_mtx;
        msgtype_thread_map_t m_boost_threads;
        msgtype_btthread_map_t m_threads;
};

class DynamicObserver: public BufferedMessageObserver
{
    // BEWARE: no type safety here, callbacks that are registered should not
    // blindly cast to the derived message type that they need, but should
    // check the result of the cast first
    typedef void (*callback_f_ptr)(boost::shared_ptr<Message const> m);
    typedef std::map<MessageType::e, callback_f_ptr> callback_map_t;
    public:
        DynamicObserver();
        virtual ~DynamicObserver();
        
        void setCallback(MessageType::e id, callback_f_ptr f);

        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}Buffered($ptrName m);
        #end for
        #end for

    private:
       callback_map_t m_callbacks;
};

class DebugMessageObserver: public MessageObserver
{
    public:
        DebugMessageObserver(unsigned int level = 1);

        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        #set $ptrName = $className + "_ptr"
        virtual void on${className}($ptrName m);
        #end for
        #end for

    protected:
        unsigned int m_level;
};


class UnknownMessageIdException : public std::exception
{
    public:
        UnknownMessageIdException(uint32_t id);
        virtual const char * what() const throw();
    protected:
        uint32_t m_id;
};

class MessageSource : public Observable<MessageObserver>
{
    public:
        void notifyObservers(const_svec_ptr bytes);
    
    // Ideally protected, but boost.python pointer_holder requires puplic
    // default constructor to be available in order to allow pointers to this
    // type
    // protected:
        MessageSource();
};

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
            return os << "MessageType::Unknown (" << int(e) << ")";
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
            return os << "$e.name::Unknown (" << int(e) << ")";
    }
}
#end for


template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Message const& m)
{
    switch (m.m_id)
    {
        #for $g in $groups
        #for $m in $g.messages
        #set $className = $m.name + "Message"
        case $m.id:
            return os << dynamic_cast<$className const&>(m);
        #end for
        #end for
        default:
            os << "Unknown message {";
            os << " id = " << std::dec << m.m_id << ",";
            os << " group = " << m.m_group;
            os << " }";
            return os;
    }
}

#for $g in $groups
#for $m in $g.messages
#set $className = $m.name + "Message"
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, $className const& #if $len($m.fields)#m#end if#)
{
    os << "$className {";
    #for i, f in $enumerate($m.fields)
    #if hasattr($f.type, "name") and ($f.type.name == "int8" or $f.type.name == "byte") 
    os << " $f.name = " << (int)m.${f.name}()#if $i < $len($m.fields) - 1# << ","#end if#;
    #elif hasattr($f.type, "name") and ($f.type.name == "string")
    os << " $f.name = \"" << m.${f.name}()#if $i < $len($m.fields) - 1# << "\","#else# << "\""#end if#;
    #else
    os << " $f.name = " << m.${f.name}()#if $i < $len($m.fields) - 1# << ","#end if#;
    #end if
    #end for
    os << " }";
    return os;
}

#end for
#end for

} // namespace cauv

\#endif // ndef __CAUV_MESSAGES_H__
