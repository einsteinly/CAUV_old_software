#include <zmq.hpp>
#include <debug/cauv_debug.h>
#include <boost/program_options.hpp>

#include <string.h>

#include "daemon.h"
#include "addresses.h"

namespace cauv {

ZmqGroupDaemon::ZmqGroupDaemon (std::string name) :
    name(name),
    zm_context(1),
    zm_pub_sock(zm_context,ZMQ_PUB),
    zm_recv_sock(zm_context,ZMQ_PULL),
    zm_control_pub(zm_context,ZMQ_PUB),
    zm_control_sub(zm_context,ZMQ_SUB) {

    zm_pub_sock.bind(get_local_sub(name).c_str());
    zm_recv_sock.bind(get_local_push(name).c_str());

    std::string multicast_addr (get_multicast_control().c_str());
    zm_control_pub.connect(multicast_addr.c_str());
    zm_control_sub.connect(multicast_addr.c_str());
    zm_control_sub.setsockopt(ZMQ_SUBSCRIBE,"",0);
}

void ZmqGroupDaemon::run(void) {
    zmq::message_t msg;
    zmq::pollitem_t items[2];
    items[0].socket = zm_control_sub;
    items[0].events = ZMQ_POLLIN;

    items[1].socket = zm_recv_sock;
    items[1].events = ZMQ_POLLIN;

    info() << "daemon for group" << name << "started";
    while(true) {
        if (zmq::poll(items,2) > 0) {
            debug(9) << "poll triggered";
            if (items[0].revents == ZMQ_POLLIN) {
                zm_control_sub.recv(&msg,0);
                debug() << "control message received";
                info() << (char*)msg.data();
            }
            if (items[1].revents == ZMQ_POLLIN) {
                zm_recv_sock.recv(&msg,0);
                zm_pub_sock.send(msg,0);
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

    cauv::ZmqGroupDaemon daemon(vars["group"].as<std::string>());

    if (vars.count("test")) {
        daemon.test();
    } else {
        daemon.run();
    }

    return 0;
}
