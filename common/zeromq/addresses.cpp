#include "addresses.h"
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <exception>
#include <debug/cauv_debug.h>

#include <cstdio>
#include <sstream>

namespace cauv {

#ifndef DEFAULT_MULTICAST_IFACE
#define DEFAULT_MULTICAST_IFACE "eth0"
#endif

static std::string get_multicast_interface(void) {
    char *interface = getenv("CAUV_MULTICAST_IFACE");
    if (!interface) {
        return std::string(DEFAULT_MULTICAST_IFACE);
    }
    return std::string(interface);
}

static std::string get_local_sub_filename(const std::string groupName) {
    return std::string("/tmp/cauv_sub_" + groupName);
}

static std::string get_local_push_filename(const std::string groupName) {
    return std::string("/tmp/cauv_push_" + groupName);
}

// !!! TODO: move to utility:
static std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if(!pipe){
        error() << "failed to popen pipe";
        return "";
    }
    char buffer[4096];
    std::string result = "";
    while(!std::feof(pipe))
        if(std::fgets(buffer, 4096, pipe))
            result += buffer;
    pclose(pipe);
    return result;
}

//!!! hacky as hell, and non-portable. have fun figuring it out on OSX
//essentially greps the open unix domain sockets for the file that would be used
bool is_daemon_running (const std::string groupName) {
    std::string sub_filename(get_local_sub_filename(groupName));
    std::string push_filename(get_local_push_filename(groupName));
    #ifdef __APPLE__
    // This was fun :) Did you know lsof does machine-readable output? I
    // didn't!
    std::stringstream domain_socket_files(exec("lsof -UFn"));
    std::string line;
    while(std::getline(domain_socket_files, line))
        if(line.substr(1) == sub_filename ||
           line.substr(1) == push_filename)
            return true;
    #else // def __APPLE__
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
    #endif // else def __APPLE__
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
    unsigned char low = 0;
    unsigned char high = 0;

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

