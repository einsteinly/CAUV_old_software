#include "workarounds.h" // _must_ be first
#include <boost/python.hpp> // oh yes :)

#include "../../generated/messages.h"
#include "../common/cauv_node.h"

#include "emit_static.h"

namespace bp = boost::python;


void cauvDebug(const char* s, int l){
    // relying on order of local destruction: LIFO
    ThreadSave guard;
    debug(l) << s;
}

void cauvDebug1(const char* s){
    ThreadSave guard;
    debug() << s;
}

void cauvWarning(const char* s){
    ThreadSave guard;
    warning() << s;
}

void cauvError(const char* s){
    ThreadSave guard;
    error() << s;
}

void cauvInfo(const char* s){
    ThreadSave guard;
    info() << s;
}

struct Blah{
    int monkey;
};

/*** Support Wrappers, etc. ***/
struct Thing{
    int callFoo(){
        return this->foo();
    }

    int getMonkey(Blah m){
        return m.monkey;
    }

    virtual int foo(){
        return 3;
    }
};

struct ThingCaller{
    void setThing(boost::shared_ptr<Thing> const& t){
        m_thing = t;
    }

    int callThing(){
        if(m_thing)
            return m_thing->foo();
        return -1;
    }

    private:
        boost::shared_ptr<Thing> m_thing;
};

struct ThingWrapper: public Thing, bp::wrapper<Thing>{
    int foo(){
        GILLock l; // GIL MUST be held during get_override!
        if(bp::override f = this->get_override("foo")){
            return f();
        }
        l.release();
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
            GILLock l; // GIL MUST be held during get_override!
            if(bp::override f = this->get_override("toBytes")){
                f();
            }
            l.release();
            assert(0);
        }
};


class CauvNodeWrapper:
    public CauvNode,
    public bp::wrapper<CauvNode>
{
    public:
        CauvNodeWrapper(std::string const& name, std::string const& server, unsigned port)
            : CauvNode(name){
            defaultOptions();
            m_server = server;
            m_port = port;
        }

        void onRun(){
            GILLock l;
            if(bp::override f = this->get_override("onRun")){
                f();
            }else{
                l.release();
                CauvNode::onRun();
            }
        }

        /*void default_onRun(){
            this->onRun();
        }*/

        void run(){
            CauvNode::run(true);
        }

        /*int foo(boost::shared_ptr<Message const> m){
            debug() << "foo called";
            debug() << m;
            std::cerr << "yes, foo was called" << std::endl;
            return 9;
        }*/

        boost::shared_ptr<ReconnectingSpreadMailbox> get_mailbox() const{
            return CauvNode::mailbox();
        }
};

/*
class AIMessageObserver:
    public BufferedMessageObserver,
    public bp::wrapper<BufferedMessageObserver>
{
    public:
        // only check for overrides of onAIMessage
        void onAIMessage(AIMessage_ptr m){
            GILLock l;
            if(bp::override f = this->get_override("onAIMessage")){
                try{
                    f(m);
                }catch(bp::error_already_set const &){
                    error() << __FILE__ << __LINE__ << ":" << __func__ << "Error in python callback:";
                    if(PyErr_Occurred()){
                        PyErr_Print();
                    }
                }
            }
            l.release();
            BufferedMessageObserver::onAIMessage(m);
        }
};
*/


class SpreadMessageWrapper:
    public SpreadMessage,
    public bp::wrapper<SpreadMessage>
{
    public:
        virtual MessageFlavour getMessageFlavour() const{
            GILLock l;
            if(bp::override f = this->get_override("getMessageFlavour")){
                f();
            }
            l.release();
            error() << "1. Spread Messages should not be exposed to Python";
            error() << "2. You should DEFINITELY not be using them";
            assert(0);
        }
};



/*** Actual Functions to Generate the Interface: ***/
void emitThing(){
    bp::class_<Blah>("Blah")
        .add_property("monkey", &Blah::monkey)
    ;
    bp::class_<ThingWrapper,
               boost::shared_ptr<ThingWrapper>,
               boost::noncopyable
              >("Thing")
        .def("callFoo", wrap(&Thing::callFoo))
        .def("foo", wrap(&Thing::foo))
    ;
    bp::class_<ThingCaller, boost::noncopyable>("ThingCaller")
        .def("setThing", &ThingCaller::setThing)
        .def("callThing", &ThingCaller::callThing)
    ;
}

void emitDebug(){
    bp::def("debug", cauvDebug);
    bp::def("debug", cauvDebug1);
    bp::def("warning", cauvWarning);
    bp::def("error", cauvError);
    bp::def("info", cauvInfo);
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
        /* does this function actually serve any useful purpose other than
         * maybe testing, and causing crashes? -- don't expose it?
         */
        .def("receive", wrap(&ReconnectingSpreadMailbox::receiveMessage))
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
               boost::noncopyable/*,
               boost::shared_ptr<Message>*/
              >("__Message", bp::no_init)
        //.add_property("group", &Message::group)
    ;
}


void emitCauvNode(){
    bp::class_<CauvNodeWrapper,
               boost::noncopyable
              >("CauvNode", bp::init<std::string, std::string, unsigned>())
         .def("run", wrap(&CauvNodeWrapper::run))
         .def("onRun", wrap(&CauvNodeWrapper::onRun))
         .def("send", wrap(&CauvNode::send))
         .def("join", wrap(&CauvNode::joinGroup))
         .def("addObserver", wrap(&CauvNode::addMessageObserver))
         .def("removeObserver", wrap(&CauvNode::removeMessageObserver))
         //.def("foo", wrap(&CauvNodeWrapper::foo))
         .add_property("mailbox", &CauvNodeWrapper::get_mailbox)
         //.add_property("monitor", &CauvNodeWrapper::get_mailboxMonitor)
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


void emitAIMessageObserver(){
    /*bp::class_<AIMessageObserver,
               boost::noncopyable,
               boost::shared_ptr<AIMessageObserver>
              >("AIMessageObserver")
        .def("onAIMessage", &wrap(BufferedMessageObserver::onAIMessage))
   ;*/
}

