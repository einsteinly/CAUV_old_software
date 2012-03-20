#include <debug/cauv_debug.h>

#include <zmq.hpp>

#include <string.h>
#include <string>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <generated/types/DebugMessage.h>
#include <generated/message_observers.h>

#include <common/zeromq/zeromq_mailbox.h>
#include <common/zeromq/zeromq_mailbox_monitor.h>
#include <common/zeromq/addresses.h>

void send_message(std::string group_name, std::string msg_str) {
    zmq::context_t test_context(1);
    zmq::socket_t zm_push_sock(test_context,ZMQ_PUSH);
    zm_push_sock.connect(std::string("ipc:///tmp/pub_" + group_name).c_str());
    zmq::message_t msg(msg_str.size() + 1);
    memcpy(msg.data(),msg_str.c_str(),msg_str.size() + 1);
    info() << "sending message: \"" << msg_str << "\" to group " << group_name;
    zm_push_sock.send(msg,ZMQ_NOBLOCK);
    info() << "sent message";
}

void receive_messages(std::string group_name) {
    zmq::context_t test_context(1);
    zmq::socket_t zm_sub_sock(test_context,ZMQ_SUB);
    zm_sub_sock.connect(std::string("ipc:///tmp/" + group_name).c_str());
    zm_sub_sock.setsockopt(ZMQ_SUBSCRIBE,"",0);
    zmq::message_t msg;
    zm_sub_sock.recv(&msg,0);
    info() << "got message : \"" << (char*)msg.data() << "\"";
}

void multicast_pub(std::string msg_str) {
    zmq::context_t test_context(1);
    zmq::socket_t zm_pub_sock(test_context,ZMQ_PUB);
    int64_t multicast_loopback = 0;
    zm_pub_sock.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    info() << "connecting to " << cauv::get_multicast_control();
    zm_pub_sock.connect(cauv::get_multicast_control().c_str());
    zmq::message_t msg(msg_str.size() + 1);
    memcpy(msg.data(),msg_str.c_str(),msg_str.size() + 1);
    info() << "sending message: \"" << msg_str << "\" to multicast";
    zm_pub_sock.send(msg,0);
    info() << "sent message";
}

void multicast_sub(void) {
    zmq::context_t test_context(1);
    zmq::socket_t zm_sub_sock(test_context,ZMQ_SUB);
    zm_sub_sock.setsockopt(ZMQ_SUBSCRIBE,"",0);
    int64_t multicast_loopback = 0;
    zm_sub_sock.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    info() << "connecting to " << cauv::get_multicast_control();
    zm_sub_sock.connect(cauv::get_multicast_control().c_str());
    zmq::message_t msg;
    zm_sub_sock.recv(&msg,0);
    info() << "got message : \"" << (char*)msg.data() << "\"";
}

void send_debug_message(std::string message) {
    cauv::ZeroMQMailbox mb;
    mb.sendMessage(boost::make_shared<cauv::DebugMessage>(cauv::DebugType::Info,message),cauv::RELIABLE_MSG);
}

class TestObserver: public cauv::MessageObserver {
    virtual void onDebugMessage(DebugMessage_ptr m) {
        debug() << "got debug message" << *m;
    }
};

void create_event_monitor(void) {
    boost::shared_ptr<cauv::ZeroMQMailbox> mb = boost::make_shared<cauv::ZeroMQMailbox>();
    mb->joinGroup("debug");
    cauv::ZeroMQMailboxEventMonitor mon(mb);
    mon.addMessageObserver(boost::make_shared<TestObserver>());
    mon.startMonitoringSync();
}

namespace po = boost::program_options;

int main(int argc, char** argv) {
    po::options_description desc("Test Options:");
    desc.add_options()
        ("help,h", "print this help message")
        ("send_message,s", po::value<std::string>(), "send message to daemon")
        ("pub_message,p", po::value<std::string>(), "publish control message")
        ("receive_message,r", "receive messages from (local) group")
        ("receive_multicast,m", "receive messages from multicast control")
        ("group,g", po::value<std::string>()->default_value("default"), "group to send message to")
        ("send_debug_msg,d",po::value<std::string>(),"create a zeromq mailbox")
        ("create_event,e", "create a zeromq event monitor")
    ;

    po::variables_map vars;
    po::store(po::parse_command_line(argc,argv,desc),vars);

    if (vars.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (vars.count("send_message")) {
        send_message(vars["group"].as<std::string>(),vars["send_message"].as<std::string>());
    }
    if (vars.count("receive_message")) {
        receive_messages(vars["group"].as<std::string>());
    }
    if (vars.count("receive_multicast")) {
        multicast_sub();
    }
    if (vars.count("pub_message")) {
        multicast_pub(vars["pub_message"].as<std::string>());
    }
    if (vars.count("send_debug_msg")) {
        send_debug_message(vars["send_debug_msg"].as<std::string>());
    }
    if (vars.count("create_event")) {
        create_event_monitor();
    }
    return 0;
}
