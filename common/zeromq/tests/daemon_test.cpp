/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <debug/cauv_debug.h>

#include <string.h>
#include <string>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <generated/types/DebugMessage.h>
#include <generated/message_observers.h>

#include <common/zeromq/zeromq_mailbox.h>

void broadcast_debug_messages(std::string message, int interval) {
    cauv::ZeroMQMailbox mb;
    mb.startMonitoringAsync();
    while (true) {
        mb.sendMessage(boost::make_shared<cauv::DebugMessage>(cauv::DebugType::Info, message), cauv::RELIABLE_MSG);
        usleep(interval * 1000);
    }
}

void send_debug_message(const std::string& message) {
    cauv::ZeroMQMailbox mb;
    mb.startMonitoringAsync();
    mb.sendMessage(boost::make_shared<cauv::DebugMessage>(cauv::DebugType::Info,message), cauv::RELIABLE_MSG);
    mb.stopMonitoring();
}

class TestObserver: public cauv::MessageObserver {
    virtual void onDebugMessage(DebugMessage_ptr m) {
        debug() << "got debug message" << *m;
    }
};

void create_event_monitor(void) {
    boost::shared_ptr<cauv::ZeroMQMailbox> mb = boost::make_shared<cauv::ZeroMQMailbox>();
    mb->joinGroup("debug");
    mb->subMessage(cauv::DebugMessage());
    mb->addMessageObserver(boost::make_shared<TestObserver>());
    mb->startMonitoringSync();
}

namespace po = boost::program_options;

int main(int argc, char** argv) {
    po::options_description desc("Test Options:");
    desc.add_options()
        ("help,h", "print this help message")
        ("create_event,e", "create a zeromq event monitor")
        ("send_debug_msg,d",po::value<std::string>(),"create a zeromq mailbox")
        ("repeat_debug_msg,r",po::value<std::string>(), "broadcast a debug message periodically")
        ("time_period,t",po::value<int>()->default_value(1000), "time between debug messages when broadcasting")
    ;

    po::variables_map vars;
    po::store(po::parse_command_line(argc,argv,desc),vars);

    if (vars.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (vars.count("send_debug_msg")) {
        send_debug_message(vars["send_debug_msg"].as<std::string>());
    }
    if (vars.count("repeat_debug_msg")) {
        broadcast_debug_messages(vars["repeat_debug_msg"].as<std::string>(),
                                 vars["time_period"].as<int>());
    }
    if (vars.count("create_event")) {
        create_event_monitor();
    }
    return 0;
}
