#include "zeromq_addresses.h"
#include <cstdlib>

#ifndef CAUV_DEFAULT_VEHICLE_NAME
#define CAUV_DEFAULT_VEHICLE_NAME "red_herring"
#endif

#ifndef CAUV_DEFAULT_IPC_DIR
#define CAUV_DEFAULT_IPC_DIR "/tmp/cauv"
#endif

namespace cauv {

std::string get_vehicle_name(void) {
    std::string vehicle_name(CAUV_DEFAULT_VEHICLE_NAME);
    char *env_vehicle_name;
    env_vehicle_name = std::getenv("CAUV_VEHICLE_NAME");
    if (env_vehicle_name) {
        vehicle_name = env_vehicle_name;
    }
    return vehicle_name;
}

std::string get_ipc_directory(std::string vehicle_name) {
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
