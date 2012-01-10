#ifndef CAUV_ZMQ_ADDRESSES_H
#define CAUV_ZMQ_ADDRESSES_H

#include <string>

namespace cauv {

bool is_daemon_running(const std::string groupName);

const std::string get_local_sub(const std::string groupName);

const std::string get_local_push(const std::string groupName);

const std::string get_multicast_control();

const std::string get_multicast_group(const std::string groupName);

}

#endif //ndef CAUV_ZMQ_ADDRESSES_H
