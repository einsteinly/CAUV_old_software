#include <zmq.hpp>
#include <boost/program_options.hpp>
#include <sstream>
#include <string.h>
#include <stdint.h>

#include <debug/cauv_debug.h>

#include "daemon.h"
#include "addresses.h"

namespace cauv {

class SocketInUseError: public std::runtime_error {
    public:
    SocketInUseError(void) : 
        std::runtime_error("tried to bind to unix domain socket alread in use") {};
};

ZmqGroupDaemon::ZmqGroupDaemon (std::string name) :
    name(name),
    has_listeners(false),
    has_remote_listeners(false),
    zm_context(1),
    zm_pub_sock(zm_context,ZMQ_PUB),
    zm_recv_sock(zm_context,ZMQ_PULL),
    zm_control_pub(zm_context,ZMQ_PUB),
    zm_control_sub(zm_context,ZMQ_SUB),
    zm_group_pub(zm_context,ZMQ_PUB),
    zm_group_sub(zm_context,ZMQ_SUB) {

    //check before binding because zmq will overwrite anything that's running,
    //causing problems
    if (is_daemon_running(name)) {
        error() << "daemon for group" << name << "is already running";
        throw SocketInUseError();
    }

    zm_pub_sock.bind(get_local_sub(name).c_str());
    zm_recv_sock.bind(get_local_push(name).c_str());

    //no multicast loopback - it carries a performance penalty and would result
    //in messages being sent twice to local nodes.
    int64_t multicast_loopback = 0;
    zm_control_pub.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    zm_control_sub.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    zm_control_sub.setsockopt(ZMQ_SUBSCRIBE,"",0);

    zm_group_pub.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    zm_group_sub.setsockopt(ZMQ_MCAST_LOOP,&multicast_loopback,sizeof(multicast_loopback));
    zm_group_sub.setsockopt(ZMQ_SUBSCRIBE,"",0);

    std::string control_multicast_addr (get_multicast_control().c_str());
    zm_control_pub.connect(control_multicast_addr.c_str());
    zm_control_sub.connect(control_multicast_addr.c_str());

    std::string group_multicast_addr(get_multicast_group(name));
    debug() << "connecting to " << group_multicast_addr;
    zm_group_pub.connect(group_multicast_addr.c_str());
    zm_group_sub.connect(group_multicast_addr.c_str());
}

void ZmqGroupDaemon::handle_control_msg(const std::string msg) {
    debug(5) << msg;
    std::istringstream iss(msg);
    std::string msg_group;
    iss >> msg_group;
    if (msg_group != name) {
        return;
    } 
    debug(5) << "message matches group";
    std::string message;
    iss >> message;
    if (message == "HAVE_LISTENERS") {
        if (!has_remote_listeners) {
            debug() << "found remote listener";
        }
        has_remote_listeners = true;
    }
    if (message == "HELLO") {
        debug() << "someone said HELLO";
        if (has_listeners) {
            debug() << "telling them we have listeners";
            send_control_msg("HAVE_LISTENERS");
        }
    }
}

void ZmqGroupDaemon::send_control_msg(const std::string msg) {
    debug(2) << "sending control message" << msg;
    std::stringstream message;
    message << name << " " << msg;
    zmq::message_t zm_msg(message.str().size() + 1);
    memcpy(zm_msg.data(),message.str().c_str(),message.str().size() + 1);
    zm_control_pub.send(zm_msg,0);
}

void ZmqGroupDaemon::run(void) {
    zmq::message_t msg;
    zmq::message_t msg2;
    zmq::pollitem_t items[3];
    items[0].socket = zm_control_sub;
    items[0].events = ZMQ_POLLIN;

    items[1].socket = zm_recv_sock;
    items[1].events = ZMQ_POLLIN;

    items[2].socket = zm_group_sub;
    items[2].events = ZMQ_POLLIN;

    info() << "daemon for group" << name << "started";
    send_control_msg("HELLO");
    while(true) {
        if (zmq::poll(items,3) > 0) {
            debug(9) << "poll triggered";
            if (items[0].revents == ZMQ_POLLIN) {
                zm_control_sub.recv(&msg,0);
                debug(4) << "control message received";
                handle_control_msg(std::string(reinterpret_cast<char*>(msg.data())));
            }
            if (items[1].revents == ZMQ_POLLIN) {
                zm_recv_sock.recv(&msg,0);
                //!!! hardcoding value for now
                if(*reinterpret_cast<const uint32_t*>(msg.data()) == 500) {
                    has_listeners = true;
                    debug() << "node joined group " << name;
                    send_control_msg("HAVE_LISTENERS");
                }
                if (has_remote_listeners) {
                    msg2.copy(&msg);
                    zm_group_pub.send(msg2,0);
                }
                if (has_listeners) {
                    zm_pub_sock.send(msg,0);
                }
            }
            if (items[2].revents == ZMQ_POLLIN) {
                zm_group_sub.recv(&msg,0);
                if (msg.size() < 4) {
                    warning() << "too-small message received, size" << msg.size();
                    continue;
                }
                if (has_listeners) {
                    zm_pub_sock.send(msg,0);
                }
            }
        }
    }
}

void ZmqGroupDaemon::test(void) {
    zmq::message_t msg;
    zm_recv_sock.recv(&msg,0);
    info() << (char*) msg.data();
}

}

namespace po = boost::program_options;

int main (int argc, char** argv) {
    po::options_description desc("Zmq Daemon Options");

    desc.add_options()
        ("help,h", "print this help message")
        ("group,g",po::value<std::string>()->default_value("default"),"group to serve")
        ("test,t", "run once and then exit")
    ;


    po::variables_map vars;
    po::store(po::parse_command_line(argc,argv,desc),vars);

    if (vars.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    debug::setProgramName(vars["group"].as<std::string>() + "_daemon");
    cauv::ZmqGroupDaemon daemon(vars["group"].as<std::string>());

    if (vars.count("test")) {
        daemon.test();
    } else {
        daemon.run();
    }

    return 0;
}
