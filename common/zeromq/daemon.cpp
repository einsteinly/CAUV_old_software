#include <zmq.hpp>
#include <debug/cauv_debug.h>
#include <boost/program_options.hpp>

#include <string.h>

#include "daemon.h"

namespace cauv {

ZmqGroup::ZmqGroup (std::string name) :
    name(name),
    zm_context(1),
    zm_pub_sock(zm_context,ZMQ_PUB),
    zm_recv_sock(zm_context,ZMQ_PULL),
    zm_control_sock(zm_context,ZMQ_REP) {

    zm_pub_sock.bind(std::string("ipc:///tmp/" + name).c_str());
    zm_recv_sock.bind(std::string("ipc:///tmp/pub_" + name).c_str());
    zm_control_sock.bind(std::string("tcp://*:13234").c_str());
}

void ZmqGroup::run(void) {
    zmq::message_t msg;
    while(true) {
        zm_recv_sock.recv(&msg,0);
        debug() << (char*) msg.data();
        zm_pub_sock.send(msg,0);
    }
}

void ZmqGroup::test(void) {
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

    cauv::ZmqGroup group(vars["group"].as<std::string>());
    info() << "waiting for message";

    if (vars.count("test")) {
        group.test();
    } else {
        group.run();
    }

    return 0;
}
