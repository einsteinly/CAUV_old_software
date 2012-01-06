#include <debug/cauv_debug.h>

#include <zmq.hpp>

#include <string.h>
#include <string>

#include <boost/program_options.hpp>

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

namespace po = boost::program_options;

int main(int argc, char** argv) {
    po::options_description desc("Test Options:");
    desc.add_options()
        ("help,h", "print this help message")
        ("send_message,s", po::value<std::string>(), "send message to daemon")
        ("receive_message,r", "receive messages from group")
        ("group,g", po::value<std::string>()->default_value("default"), "group to send message to")
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
    return 0;
}
