#include "zeromq_mailbox.h"
#include "addresses.h"
#include <debug/cauv_debug.h>
#include <generated/types/message.h>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>

namespace cauv {

ZeroMQGroup::ZeroMQGroup(zmq::context_t &context, const std::string name) :
    push_skt(context,ZMQ_PUSH),
    sub_skt(context,ZMQ_SUB),
    name(name) {

    sub_skt.setsockopt(ZMQ_SUBSCRIBE,"",0);
    push_skt.connect(get_local_push(name).c_str());
    sub_skt.connect(get_local_sub(name).c_str());

    poll_item.socket = sub_skt;
    poll_item.events = ZMQ_POLLIN;
}
    

ZeroMQMailbox::ZeroMQMailbox(void) :
    zm_context(1), m_group_gen(0) {

}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message, MessageReliability reliablility) {
    return send_message_to_group(message,zm_groups[message->group()]);
}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message, MessageReliability reliability,
                               const std::string &destinationGroup) {
    return send_message_to_group(message,zm_groups[destinationGroup]);
}

void ZeroMQMailboxDeleteSvecptr (void *data, void *hint) {
    debug(14) << "deleting shared pointer to data";
    const_svec_ptr *vec = (const_svec_ptr*) hint;
    delete vec;
}

int ZeroMQMailbox::send_message_to_group(boost::shared_ptr<const Message> message, boost::shared_ptr<ZeroMQGroup> group) {
    debug(14) << "sending data to group";
    const_svec_ptr bytes = message->toBytes();

    //all kinds of fun combining memory management schemes here...
    //this does avoid a potentially expensive copy though.
    void *zmq_shared_ptr = new const_svec_ptr(bytes);
    zmq::message_t msg((void*)bytes->data(), bytes->size(), ZeroMQMailboxDeleteSvecptr, zmq_shared_ptr);

    group->push_skt.send(msg,0);
    debug(14) << "data sent";
    return 0;
}

void ZeroMQMailbox::joinGroup (const std::string &groupName) {
    if (zm_groups.count(groupName)) {
        warning() << "tried to join group " << groupName << " twice!";
        return;
    }
    
    zm_groups[groupName] = boost::make_shared<ZeroMQGroup>(boost::ref(zm_context),groupName);
    m_group_gen++;
}

void ZeroMQMailbox::leaveGroup (const std::string &groupName) {
    if (!zm_groups.erase(groupName)) {
        warning() << "tried to leave group " << groupName << " which hasn't been joined yet!";
    }
    m_group_gen++;
}

std::vector <boost::shared_ptr <ZeroMQGroup> >
ZeroMQMailbox :: get_groups(void) {
    std::vector <boost::shared_ptr <ZeroMQGroup> > group_vec;
    BOOST_FOREACH(zm_group_map::value_type group_pair, zm_groups) {
        group_vec.push_back(group_pair.second);
    }
    return group_vec;
}

}
