#include "workarounds.h" // _must_ be first
#include <boost/python.hpp>
#include <boost/utility.hpp>

#include <debug/cauv_debug.h>
#include <common/cauv_node.h>
#include <common/mailbox.h>
#include <common/mailbox.h>
#include <common/msg_classes/bounded_float.h>
#include <generated/message_observers.h>
#include <generated/types/message.h>
#include <generated/types/MembershipChangedMessage.h>

#include <string>
#include <cstring>

#include "emit_static.h"

namespace bp = boost::python;
using namespace cauv;

//python list -> std::string[] -> c string (char**)

class pyListWrapper : boost::noncopyable{
    public:
    pyListWrapper(bp::list &list) :
        length(bp::len(list)),
        char_list(new char* [sizeof(char*) * length]) {

        for (bp::ssize_t i = 0; i < length; i++) {
            std::string str = bp::extract<std::string>(list[i]);
            char_list[i] = new char [str.length() + 1];
            strncpy(char_list[i],str.c_str(),str.length() + 1);
        }
    }
    char ** get() {
        return length? char_list : NULL;
    }
    bp::ssize_t len() {
        return length;
    }
    ~pyListWrapper() {
        for (bp::ssize_t i = 0; i < length; i++) {
            delete[] char_list[i];
        }
        delete[] char_list;
    }
    private:
    bp::ssize_t length;
    char **char_list;
};

// NB: don't yield the GIL (otherwise it's possible *s could become invalid)
void cauvDebug(const char* s, int l){
    debug(l) << s;
}

void cauvDebug1(const char* s){
    debug() << s;
}

void cauvWarning(const char* s){
    warning() << s;
}

void cauvError(const char* s){
    error() << s;
}

void cauvInfo(const char* s){
    info() << s;
}

void setDebugName(const char* s){
    debug::setProgramName(s);
}

int debugParseOptions(bp::list &argv){
    pyListWrapper argv_w(argv);
    return debug::parseOptions(argv_w.len(),argv_w.get());
}

#ifdef EMIT_SILLY_BOOSTPYTHON_TEST_STRUCTURES
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
#endif // def EMIT_SILLY_BOOSTPYTHON_TEST_STRUCTURES


class MessageWrapper:
    public Message,
    public bp::wrapper<Message>
{
    public:
        const_svec_ptr toBytes() const{
            GILLock l; // GIL MUST be held during get_override!
            if(bp::override f = this->get_override("toBytes")){
                return f();
            }
            l.release();
            return Message::toBytes();
        }
};

class CauvNodeWrapper:
    public CauvNode,
    public bp::wrapper<CauvNode>
{
    public:
        CauvNodeWrapper(std::string const& name)
            : CauvNode(name){
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

        void run(bool synchronous){
            CauvNode::run(synchronous);
        }

        int parseOptions (bp::list& argv) {
            pyListWrapper argv_w(argv);
            return CauvNode::parseOptions(argv_w.len(),argv_w.get());
        }

        /*int foo(boost::shared_ptr<Message const> m){
            debug() << "foo called";
            debug() << m;
            std::cerr << "yes, foo was called" << std::endl;
            return 9;
        }*/

        boost::shared_ptr<Mailbox> get_mailbox() const{
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

std::vector<uint8_t> mkByteVec(std::string const& b16encoded){
    std::vector<uint8_t> r;
    r.reserve(b16encoded.size()/2);
    const static uint8_t nibble_lookup[256] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0,
        0,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    for(uint32_t i = 0; i < b16encoded.size()-1; i+= 2)
        r.push_back((nibble_lookup[int(b16encoded[i])] << 4) | nibble_lookup[int(b16encoded[i+1])]);
    return r;
}

#if EMIT_SILLY_BOOSTPYTHON_TEST_STRUCTURES
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
#else
void emitThing(){}
#endif // def EMIT_SILLY_BOOSTPYTHON_TEST_STRUCTURES

void emitDebug(){
    bp::def("debug", cauvDebug);
    bp::def("debug", cauvDebug1);
    bp::def("warning", cauvWarning);
    bp::def("error", cauvError);
    bp::def("info", cauvInfo);
    bp::def("setDebugName", setDebugName);
    bp::def("debugParseOptions", debugParseOptions);
}

void emitMailbox(){
    /* need to explicitly resolve pointer to overloaded function: */
    typedef int (Mailbox::*sm_ptr3_t)(
        boost::shared_ptr<const Message>, MessageReliability, std::string const&
    );
    bp::class_<Mailbox,
               boost::noncopyable,
               boost::shared_ptr<Mailbox>
              >("Mailbox", bp::no_init)
        .def("send", wrap((sm_ptr3_t) &Mailbox::sendMessage))
    ;

    bp::class_<MessageSource,
               boost::noncopyable,
               boost::shared_ptr<MessageSource>
              >("__MessageSourceBase")
    ;

    bp::enum_<MessageReliability>("MessageReliability")
        .value("RELIABLE_MSG",RELIABLE_MSG)
        .value("UNRELIABLE_MSG",UNRELIABLE_MSG)
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
              >("CauvNode", bp::init<std::string>())
         .def("run", wrap(&CauvNodeWrapper::run))
         .def("stop", wrap(&CauvNode::stopNode))
         .def("onRun", wrap(&CauvNodeWrapper::onRun))
         .def("send", wrap(&CauvNode::send))
         .def("join", wrap(&CauvNode::joinGroup))
         .def("addObserver", wrap(&CauvNode::addMessageObserver))
         .def("removeObserver", wrap(&CauvNode::removeMessageObserver))
         .def("parseOptions", &CauvNodeWrapper::parseOptions)
         //.def("foo", wrap(&CauvNodeWrapper::foo))
         .add_property("mailbox", &CauvNodeWrapper::get_mailbox)
         //.add_property("monitor", &CauvNodeWrapper::get_mailboxMonitor)
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

void emitPostGenerated(){
    bp::def("mkByteVec", mkByteVec);
    
    // lacking in doc comments...
    bp::class_<BoundedFloat,
               bp::bases<BoundedFloatBase>,
               boost::shared_ptr<BoundedFloat> 
              >("BoundedFloat",bp::init<float,float,float,BoundedFloatType::e>())
        ;
}
