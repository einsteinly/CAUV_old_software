/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


/***  This is a generated file, do not edit ***/
\#ifndef __CAUV_MESSAGE_OBSERVERS_H__
\#define __CAUV_MESSAGE_OBSERVERS_H__

\#include <boost/shared_ptr.hpp>

\#include <utility/observable.h>
\#include <utility/serialisation.h>

\#include "types/message_type.h"

// Forward Declarations outside cauv namespace:
namespace boost{
class thread;
class shared_mutex;
} // namespace boost

namespace cauv{

class Message;
#for $g in $groups
#for $m in $g.messages
class ${m.name}Message;
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
    
    // Ideally protected, but boost.python pointer_holder requires public
    // default constructor to be available in order to allow pointers to this
    // type
    // protected:
        MessageSource();
    private:
        bool reject_invalid;
};

}

\#endif // ndef __CAUV_MESSAGE_OBSERVERS_H__
