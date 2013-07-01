/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "zeromq_addresses.h"
#include <cstdlib>
#include <xs.h>

#ifndef CAUV_DEFAULT_VEHICLE_NAME
#define CAUV_DEFAULT_VEHICLE_NAME "red_herring"
#endif

#ifndef CAUV_DEFAULT_IPC_DIR
#define CAUV_DEFAULT_IPC_DIR "/tmp/cauv"
#endif

namespace cauv {

//copied from libxs internal wire.hpp, joyfully
#define XS_CMD_SUBSCRIBE 1
#define XS_CMD_UNSUBSCRIBE 2

std::string gen_subscription_message(subscription_vec_t sub) {
    std::string sub_buf;
    uint16_t command;
    if (sub.first) {
        command = XS_CMD_SUBSCRIBE;
    } else {
        command = XS_CMD_UNSUBSCRIBE;
    }
    uint16_t filter_id = XS_FILTER_PREFIX;
    sub_buf += (char)(command >> 8);
    sub_buf += (char)(command);
    sub_buf += (char)(filter_id >> 8);
    sub_buf += (char)(filter_id);
    for (subscription_vec_t::second_type::const_iterator it = sub.second.begin();
         it != sub.second.end(); it++) {
        sub_buf += std::string(reinterpret_cast<const char*>(&(*it)), sizeof(*it));
    }
    return sub_buf;
}

subscription_vec_t parse_subscription_message(const std::string& msg) {
    subscription_vec_t vec;
    vec.first = false;
    uint16_t command;
    if (msg.size() < sizeof(command) * 2) {
        return vec;
    }
    command = (msg[0] << 8) + msg[1];
    if (command == XS_CMD_SUBSCRIBE) {
        vec.first = true;
    } else if (command == XS_CMD_UNSUBSCRIBE) {
        vec.first = false;
    }

    if (msg.size() % sizeof(uint32_t) != 0) {
        return vec;
    }
    for(uint32_t pos = sizeof(command) * 2; pos < msg.size(); pos += sizeof(uint32_t)) {
        vec.second.push_back(*reinterpret_cast<const uint32_t*>(msg.c_str() + pos));
    }
    return vec;
}

std::string get_vehicle_name(void) {
    std::string vehicle_name(CAUV_DEFAULT_VEHICLE_NAME);
    char *env_vehicle_name;
    env_vehicle_name = std::getenv("CAUV_VEHICLE_NAME");
    if (env_vehicle_name) {
        vehicle_name = env_vehicle_name;
    }
    return vehicle_name;
}

std::string get_ipc_directory(const std::string& vehicle_name) {
    std::string directory(CAUV_DEFAULT_IPC_DIR);
    char *env_ipc_directory;
    env_ipc_directory = std::getenv("CAUV_IPC_DIR");
    if (env_ipc_directory) {
        directory = env_ipc_directory;
    }
    directory += "/" + vehicle_name; 
    return directory;
}

std::string get_ipc_directory(void) {
    return get_ipc_directory(get_vehicle_name());
}

}
