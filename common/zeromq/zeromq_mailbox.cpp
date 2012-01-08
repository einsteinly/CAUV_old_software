#include "zeromq_mailbox.h"
#include "addresses.h"
#include <debug/cauv_debug.h>
#include <generated/types/message.h>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>


namespace cauv {

ZeroMQGroup::ZeroMQGroup(zmq::context_t &context, const std::string name, bool write_only) :
    push_skt(context,ZMQ_PUSH),
    sub_skt(context,ZMQ_SUB),
    name(name) {

    sub_skt.setsockopt(ZMQ_SUBSCRIBE,"",0);
    int linger = 1000;
    push_skt.setsockopt(ZMQ_LINGER,&linger,sizeof(linger));
    push_skt.connect(get_local_push(name).c_str());
    sub_skt.connect(get_local_sub(name).c_str());

    poll_item.socket = sub_skt;
    poll_item.events = ZMQ_POLLIN;
}
    

ZeroMQMailbox::ZeroMQMailbox(void) :
    zm_context(1), m_group_gen(0) {

}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message, MessageReliability reliablility) {
    return send_message_to_group(message,message->group());
}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message, MessageReliability reliability,
                               const std::string &destinationGroup) {
    return send_message_to_group(message,destinationGroup);
}

void ZeroMQMailboxDeleteSvecptr (void *data, void *hint) {
#ifdef CAUV_DEBUG_MESSAGES
    debug(11) << "deleting shared pointer to data";
#endif
    const_svec_ptr *vec = reinterpret_cast<const_svec_ptr*>(hint);
    delete vec;
}

int ZeroMQMailbox::send_message_to_group(boost::shared_ptr<const Message> message, const std::string &groupName) {
    boost::shared_ptr<ZeroMQGroup> group;

    if (!zm_groups.count(groupName)) {
        debug(3) << "adding write-only group" << groupName;
        zm_groups[groupName] = boost::make_shared<ZeroMQGroup>(boost::ref(zm_context),groupName,true);
        m_group_gen++;
    }

    group = zm_groups[groupName];

#ifdef CAUV_DEBUG_MESSAGES
    debug(9) << "sending message to group" << groupName;
    debug(10) << "message:" << &message;
#endif
    const_svec_ptr bytes = message->toBytes();

    //wait until we can send the message non-blocking
    zmq::pollitem_t pollitem;
    pollitem.socket = group->push_skt;
    pollitem.events = ZMQ_POLLOUT;
    bool keep_waiting = true;
    while (keep_waiting) {
        try {
            if (zmq::poll(&pollitem,1) > 0) {
                keep_waiting = false;
            }
        } catch (zmq::error_t err) {
            if (err.num() != EINTR) {
                throw err;
            }
        }
    }

    //all kinds of fun combining memory management schemes here...
    //this does avoid a potentially expensive copy though.
    //
    //construct a dynamically allocated shared pointer to the data so it doesn't
    //get freed until zeromq is done with it, at which point the shared pointer
    //is deleted by ZeroMQMailboxDeleteSvecptr()
    void *zmq_shared_ptr = reinterpret_cast<void*> (new const_svec_ptr(bytes));

    //!!! ugly cast to remove constness
    zmq::message_t msg(const_cast<void*>(reinterpret_cast<const void*>(bytes->data())),
                       bytes->size(), ZeroMQMailboxDeleteSvecptr, zmq_shared_ptr);

    if(!group->push_skt.send(msg,ZMQ_NOBLOCK)) {
        //if it doesn't send, delete the shared pointer
        warning() << "message failed to send without blocking despite poll";
        ZeroMQMailboxDeleteSvecptr(NULL,zmq_shared_ptr);
    }

#ifdef CAUV_DEBUG_MESSAGES
    debug(11) << "data sent";
#endif
    return 0;
}

void ZeroMQMailbox::joinGroup (const std::string &groupName) {
    if (zm_groups.count(groupName) && zm_groups[groupName]->write_only == false) {
        warning() << "tried to join group " << groupName << " twice!";
        return;
    }
    debug() << "joining group" << groupName;
    
    zm_groups[groupName] = boost::make_shared<ZeroMQGroup>(boost::ref(zm_context),groupName);
    m_group_gen++;
}

void ZeroMQMailbox::leaveGroup (const std::string &groupName) {
    if (!zm_groups.erase(groupName)) {
        warning() << "tried to leave group " << groupName << " which hasn't been joined yet!";
    }
    debug() << "leaving group" << groupName;
    m_group_gen++;
}

std::vector <boost::shared_ptr <ZeroMQGroup> >
ZeroMQMailbox :: get_groups(void) {
    std::vector <boost::shared_ptr <ZeroMQGroup> > group_vec;
    BOOST_FOREACH(zm_group_map::value_type group_pair, zm_groups) {
        if (!group_pair.second->write_only) {
            group_vec.push_back(group_pair.second);
        }
    }
    return group_vec;
}

}
