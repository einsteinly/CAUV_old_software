#include "addresses.h"
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <exception>
#include <debug/cauv_debug.h>

namespace cauv {

#ifndef DEFAULT_MULTICAST_IFACE
#define DEFAULT_MULTICAST_IFACE "eth0"
#endif

const std::string get_multicast_interface(void) {
    char *interface = getenv("CAUV_MULTICAST_IFACE");
    if (!interface) {
        return std::string(DEFAULT_MULTICAST_IFACE);
    }
    return std::string(interface);
}

const std::string get_local_sub_filename(const std::string groupName) {
    return std::string("/tmp/cauv_sub_" + groupName);
}

const std::string get_local_push_filename(const std::string groupName) {
    return std::string("/tmp/cauv_push_" + groupName);
}

//!!! hacky as hell, and non-portable. have fun figuring it out on OSX
//essentially greps the open unix domain sockets for the file that would be used
bool is_daemon_running (const std::string groupName) {
    std::string sub_filename(get_local_sub_filename(groupName));
    std::string push_filename(get_local_push_filename(groupName));
    std::fstream unix_file("/proc/net/unix",std::fstream::in);
    if (unix_file.fail()) {
        warning() << "could not open /proc/net/unix, so can't tell if anything is bound to the sockets we want to use. assuming there isn't";
        return false;
    }
    std::string word;
    while (!unix_file.eof()) {
        unix_file >> word;
        if (word == sub_filename) {
            return true;
        }
        if (word == push_filename) {
            return true;
        }
    }
    return false;
}

const std::string get_local_sub(const std::string groupName) {
    return std::string("ipc://" + get_local_sub_filename(groupName));
}

const std::string get_local_push(const std::string groupName) {
    return std::string("ipc://" + get_local_push_filename(groupName));
}

const std::string get_multicast_control() {
    std::stringstream addr;
    addr << "epgm://" << get_multicast_interface() << ";239.192.1.1:5555";
    return addr.str();
}

/* simple hash of group name to port and multicast ip address.
 * Should be sufficiently unique to avoid collisions (which are non-fatal
 * anyway, just inefficient)
 */
const std::string get_multicast_group(const std::string groupName) {
    unsigned char low;
    unsigned char high;

    for (unsigned int i = 0; i < groupName.size(); i++) {
        if (i%2) {
            low ^= groupName[i];
        } else {
            high ^= groupName[i];
        }
    }

    std::stringstream addr;
    addr << "epgm://" << get_multicast_interface() << ";239.192.";
    addr << (int)high << "." << (int)low;

    int port;
    port = (int)low + ((int)high << 8);

    port |= 1024;

    addr << ":" << port;
    return addr.str();
}

}

