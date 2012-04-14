#ifndef ZEROMQ_ADDRESSES_H
#define ZEROMQ_ADDRESSES_H
#include <string>

namespace cauv {

#define NODE_MARKER_MSGID 223
#define DAEMON_MARKER_MSGID 224

std::string get_vehicle_name(void);

std::string get_ipc_directory(void);

std::string get_ipc_directory(const std::string vehicle_name);

}

#endif
