#ifndef CAUV_ZMQ_ADDRESSES_H
#define CAUV_ZMQ_ADDRESSES_H

#include <string>

namespace cauv {

const std::string get_local_sub(const std::string groupName) {
    return std::string("ipc:///tmp/" + groupName);
}

const std::string get_local_push(const std::string groupName) {
    return std::string("ipc:///tmp/pub_" + groupName);
}

const std::string get_multicast_control() {
    return std::string("epgm://eth0;239.192.1.1:5555");
}

}

#endif //ndef CAUV_ZMQ_ADDRESSES_H
