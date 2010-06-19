#include "workarounds.h" // _must_ be first
#include <boost/python.hpp> // oh yes :)

#include "../common/messages.h"
#include "../common/cauv_node.h"

#include "emit_static.h"

namespace bp = boost::python;


/*** Support Wrappers, etc. ***/
struct Thing{
    int callFoo(){
        return this->foo();
    }

    virtual int foo(){
        return 3;
    }
};

struct ThingWrapper: public Thing, bp::wrapper<Thing>{
    int foo(){
        if(bp::override f = this->get_override("foo")){
            GILLock l; // aquire GIL before calling back into python
            return f();
        }
        return Thing::foo();
    }

    // NB: boost python falls over if wrapped default functions are passed - it
    // can't correctly determine the type, but we don't need them anyway
    /*int default_foo(){
        return this->Thing::foo();
    }*/
};


class MessageWrapper:
    public Message,
    public bp::wrapper<Message>
{
    public:
        boost::shared_ptr<const byte_vec_t> toBytes() const{
            if(bp::override f = this->get_override("toBytes")){
                GILLock l; 
                f();
            }
            assert(0);
        }
};


class CauvNodeWrapper:
    public CauvNode,
    public bp::wrapper<CauvNode>
{
    public:
        CauvNodeWrapper(std::string const& name, std::string const& host="16707@localhost")
            : CauvNode(name, host.c_str()){
        }
        
        void onRun(){
            if(bp::override f = this->get_override("onRun")){
                GILLock l; 
                f();
            }else
                CauvNode::onRun();
        }

        /*void default_onRun(){
            this->onRun();
        }*/

        int foo(boost::shared_ptr<Message const> m){
            debug() << "foo called";
            debug() << m;
            std::cerr << "yes, foo was called" << std::endl;
            return 9;
        }

        boost::shared_ptr<ReconnectingSpreadMailbox> get_mailbox() const{
            return CauvNode::mailbox();
        }
        boost::shared_ptr<MsgSrcMBMonitor> get_mailboxMonitor() const{
            return CauvNode::mailboxMonitor();
        }
};


class SpreadMessageWrapper:
    public SpreadMessage,
    public bp::wrapper<SpreadMessage>
{
    public:
        virtual MessageFlavour getMessageFlavour() const{
            if(bp::override f = this->get_override("getMessageFlavour")){
                GILLock l; 
                f();
            }
            assert(0);
        }
};



/*** Actual Functions to Generate the Interface: ***/
void emitThing(){
    bp::class_<ThingWrapper, boost::noncopyable>("Thing")
        .def("callFoo", wrap(&Thing::callFoo))
        .def("foo", wrap(&Thing::foo))
    ;
}


void emitMailbox(){
    /* need to explicitly resolve pointer to overloaded function: */
    typedef int (ReconnectingSpreadMailbox::*sm_ptr3_t)(
        boost::shared_ptr<const Message>, Spread::service, std::string const&
    );
    bp::class_<ReconnectingSpreadMailbox,
               boost::noncopyable,
               boost::shared_ptr<ReconnectingSpreadMailbox>
              >("Mailbox", bp::no_init)
        .def("join", wrap(&ReconnectingSpreadMailbox::joinGroup))
        .def("send", wrap((sm_ptr3_t) &ReconnectingSpreadMailbox::sendMessage))
        /*.def("receive", wrap(&ReconnectingSpreadMailbox::receiveMessage))
         * this function actually serves no useful purpose other than maybe
         * testing, and posisibly causing crashes -- so don't expose it
         */
    ;

    bp::class_<MessageSource,
               boost::noncopyable,
               boost::shared_ptr<MessageSource>
              >("__MessageSourceBase")
    ;

    bp::class_<MsgSrcMBMonitor,
               bp::bases<MessageSource>, // also MailboxObserver, but that is never instantiated
               boost::noncopyable,
               boost::shared_ptr<MsgSrcMBMonitor>
              >("Monitor", bp::no_init)
        .def("addObserver", wrap(&MessageSource::addObserver)) // addObserver is a member of base class MessageSource
    ;
}


void emitMessage(){
    bp::class_<MessageWrapper,
               boost::noncopyable,
               boost::shared_ptr<Message>
              >("__Message", bp::no_init)
    ;
}


void emitCauvNode(){
    bp::class_<CauvNodeWrapper,
               boost::noncopyable
              >("CauvNode", bp::init<std::string, std::string>())
         .def("run", wrap(&CauvNode::run))
         .def("onRun", wrap(&CauvNode::onRun))
         .def("foo", wrap(&CauvNodeWrapper::foo))
         .add_property("mailbox", &CauvNodeWrapper::get_mailbox)
         .add_property("monitor", &CauvNodeWrapper::get_mailboxMonitor)
    ;
}


void emitSpreadMessage(){
    bp::class_<SpreadMessageWrapper,
               boost::noncopyable,
               boost::shared_ptr<SpreadMessage>
              >("__SpreadMessage", bp::no_init)
        .def("getMessageFlavour", wrap(&SpreadMessageWrapper::getMessageFlavour))
    ;

    bp::class_<RegularMessage,
               bp::bases<SpreadMessage>,
               boost::shared_ptr<RegularMessage>
              >("__RegularMessage", bp::no_init)
    ;

    bp::class_<MembershipMessage,
               bp::bases<SpreadMessage>,
               boost::shared_ptr<MembershipMessage>
              >("__MembershipMessage", bp::no_init)
    ;
}

