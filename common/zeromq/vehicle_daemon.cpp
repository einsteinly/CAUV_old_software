
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>

#include <vector>
#include <map>
#include <set>
#include <utility>

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <zmq.h>
#include <assert.h>
#include <stdint.h>

#include "daemon_util.h"
#include "zeromq_addresses.h"

class DaemonContext {
    public:
    DaemonContext(const std::string vehicle_name, const std::string working_directory);
    void run(void);
    ~DaemonContext(void);
    private:
    bool running;
    const std::string working_directory;
    const std::string vehicle_name;
    void *zmq_context;
    void *control_rep;
    void *network_xsub;
    void *network_xpub;
    void *local_xsub;
    void *local_xpub;

    typedef std::vector<std::string> conn_list_t;
    conn_list_t local_connections;
    conn_list_t local_binds;
    conn_list_t net_connections;
    conn_list_t net_binds;

    typedef std::set<uint32_t> subscriptions_t;
    typedef std::map<uint32_t,std::pair <unsigned int, unsigned long int> > stats_t;

    subscriptions_t subs_net_to_local;
    subscriptions_t subs_local_to_net;

    stats_t stats_net_to_local;
    stats_t stats_local_to_net;

    void pump_message(void *from, void *to, stats_t *stats, subscriptions_t *subs);
    zmq_msg_t message_buf;

    void handle_control_message(void);
    static std::string handle_connect_message(void *socket, std::string connect, conn_list_t &conn_list);
    static std::string handle_bind_message(void *socket, std::string connect, conn_list_t &conn_list);
    static std::string get_conn_list_string(const conn_list_t conn_list);
    static std::string get_stats_string(const stats_t stats);
    static std::string get_subs_string(subscriptions_t subs);

    DaemonContext &operator=(const DaemonContext &other);
    DaemonContext (const DaemonContext &other);
};

DaemonContext::DaemonContext(const std::string vehicle_name, const std::string working_directory) :
    running(true),
    working_directory(working_directory),
    vehicle_name(vehicle_name) {
    zmq_context = zmq_ctx_new();
    assert(zmq_context);

    int linger = 300;
    control_rep = zmq_socket(zmq_context,ZMQ_REP);
    network_xsub = zmq_socket(zmq_context,ZMQ_XSUB);
    network_xpub = zmq_socket(zmq_context,ZMQ_XPUB);
    local_xsub = zmq_socket(zmq_context,ZMQ_XSUB);
    local_xpub = zmq_socket(zmq_context,ZMQ_XPUB);
    assert(control_rep);
    assert(network_xsub);
    assert(network_xpub);
    assert(local_xsub);
    assert(local_xpub);
    assert(zmq_setsockopt(control_rep, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(network_xsub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(network_xpub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(local_xsub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(local_xpub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    std::string control_path = "ipc://" + working_directory + "/control";
    assert(zmq_bind(control_rep,control_path.c_str()) == 0);
    std::string local_sub_path = "ipc://" + working_directory + "/sub";
    assert(zmq_bind(local_xsub, local_sub_path.c_str()) == 0);

    assert(zmq_msg_init(&message_buf) == 0);
    std::string sub_buf;
    uint32_t msgid = DAEMON_MARKER_MSGID;
    sub_buf += "\1";
    sub_buf += std::string(reinterpret_cast<char*>(&msgid),sizeof(msgid));
    assert(zmq_send(local_xsub, sub_buf.c_str(), sub_buf.size(), 0) >= 0);
}

DaemonContext::~DaemonContext(void) {
    zmq_close(control_rep);
    zmq_close(network_xsub);
    zmq_close(network_xpub);
    zmq_close(local_xsub);
    zmq_close(local_xpub);
    zmq_ctx_destroy(zmq_context);
}

void DaemonContext::pump_message(void *from, void *to, stats_t *stats, subscriptions_t *subs) {
    int flags = 0;
    if (zmq_recvmsg(from, &message_buf, ZMQ_DONTWAIT) < 0) {
        assert(errno & (EAGAIN | EINTR));
    } else {
        if (zmq_msg_more(&message_buf)) {
            flags = ZMQ_SNDMORE;
        }
        if (stats) {
            if (zmq_msg_size(&message_buf) >= sizeof(uint32_t)) {
                const uint32_t msg_id = *reinterpret_cast<const uint32_t*>(zmq_msg_data(&message_buf));
                if (msg_id == NODE_MARKER_MSGID) {
                    return;
                }
                (*stats)[msg_id].first++;
                (*stats)[msg_id].second += zmq_msg_size(&message_buf);
            }
        }
        if (subs) {
            if (zmq_msg_size(&message_buf) >= sizeof(uint32_t) + 1) {
                const char sub = *reinterpret_cast<const char*>(zmq_msg_data(&message_buf));
                const uint32_t msg_id = *reinterpret_cast<const uint32_t*>(
                                            reinterpret_cast<const char*>(zmq_msg_data(&message_buf)) + 1);
                if (sub) {
                    subs->insert(msg_id);
                } else {
                    subs->erase(msg_id);
                }
            }
        }

        if (zmq_sendmsg(to, &message_buf, flags) < 0) {
            assert(errno & (EAGAIN | EINTR));
        }
    } 
}

void DaemonContext::run(void) {
    zmq_pollitem_t sockets[] = {
        { local_xsub,
          0, ZMQ_POLLIN, 0
        },
        { local_xpub,
          0, ZMQ_POLLIN, 0
        },
        { network_xsub,
          0, ZMQ_POLLIN, 0
        },
        { network_xpub,
          0, ZMQ_POLLIN, 0
        },
        { control_rep,
          0, ZMQ_POLLIN, 0
        }
    };
    while(running) {
        int rc = zmq_poll(sockets, sizeof(sockets)/sizeof(sockets[0]), 10000);
        if (rc < 0) {
            switch(errno) {
                case ETERM:
                    running = false;
                    break;
                case EFAULT:
                default:
                    fprintf(stderr,"Error calling poll()");
                    assert(false);
                    break;
                case EINTR:
                    break;
            }
        } else {
            if (sockets[0].revents & ZMQ_POLLIN) {
                pump_message(local_xsub, network_xpub, &stats_local_to_net, NULL);
            }
            if (sockets[1].revents & ZMQ_POLLIN) {
                pump_message(local_xpub, network_xsub, NULL, &subs_net_to_local);
            }
            if (sockets[2].revents & ZMQ_POLLIN) {
                pump_message(network_xsub, local_xpub, &stats_net_to_local, NULL);
            }
            if (sockets[3].revents & ZMQ_POLLIN) {
                pump_message(network_xpub, local_xsub, NULL, &subs_local_to_net);
            }
            if (sockets[4].revents & ZMQ_POLLIN) {
                handle_control_message();
            }
        }
    }
}

std::string DaemonContext::handle_connect_message(void *socket, std::string connection, conn_list_t &conn_list) {
    std::string reply;
    if (zmq_connect(socket, connection.c_str()) < 0) {
        reply = "FAILED ";
        reply += zmq_strerror(errno);
    } else {
        reply = "SUCCESS";
        conn_list.push_back(connection);
    }
    return reply;
}

std::string DaemonContext::handle_bind_message(void *socket, std::string bind, conn_list_t &conn_list) {
    std::string reply;
    if (zmq_bind(socket, bind.c_str()) < 0) {
        reply = "FAILED ";
        reply += zmq_strerror(errno);
    } else {
        reply = "SUCCESS";
        conn_list.push_back(bind);
    }
    return reply;
}

std::string DaemonContext::get_conn_list_string(const conn_list_t conn_list) {
    std::string list;
    list += "(";
    for(conn_list_t::const_iterator it = conn_list.begin();
        it != conn_list.end(); it++) {
        list += *it;
        if (it != conn_list.end() - 1) {
            list += "; ";
        }
    }
    list += ")";
    return list;
}

std::string DaemonContext::get_stats_string(const stats_t stats) {
    std::ostringstream oss;
    oss << "(";
    for(stats_t::const_iterator it = stats.begin();
        it != stats.end(); it++) {
        oss << it->first << ": (" << it->second.first << ", " << it->second.second << ")";
        oss << "; ";
    }
    oss << ")";
    return oss.str();
}

void DaemonContext::handle_control_message(void) {
    zmq_msg_t message;
    assert(zmq_msg_init(&message) == 0);
    if (zmq_recvmsg(control_rep, &message, ZMQ_DONTWAIT) < 0) {
        if (errno & EFSM) {
            //try to clear state
            zmq_send(control_rep, "", 0, ZMQ_DONTWAIT);
        } else {
            assert(errno & (EAGAIN | EINTR));
        }
        return;
    }
    std::string control_message(reinterpret_cast<char*> (zmq_msg_data(&message)),
                                zmq_msg_size(&message));
    std::istringstream ctrl_iss(control_message);
    std::string command;
    std::string reply;
    std::string arg;
    ctrl_iss >> command;
    ctrl_iss >> arg;
    if (command == "NET_CONNECT") {
        reply = handle_connect_message(network_xpub, arg, net_connections);
    } else if (command == "NET_BIND") {
        reply = handle_bind_message(network_xsub, arg, net_binds);
    } else if (command == "LOCAL_CONNECT") {
        reply = handle_connect_message(local_xpub, arg, local_connections);
    } else if (command == "LOCAL_BIND") {
        reply = handle_bind_message(local_xsub, arg, local_binds);
    } else if (command == "STATS") {
        reply += "NET_TO_LOCAL";
        reply += get_stats_string(stats_net_to_local);
        reply += ", \n";
        reply += "LOCAL_TO_NET";
        reply += get_stats_string(stats_local_to_net);
    } else if (command == "SUBS") {
    } else if (command == "CONNECTIONS") {
        reply += "LOCAL_BINDS";
        reply += get_conn_list_string(local_binds);
        reply += ", \n";
        reply += "LOCAL_CONNECTIONS";
        reply += get_conn_list_string(local_connections);
        reply += ", \n";
        reply += "NET_BINDS";
        reply += get_conn_list_string(net_binds);
        reply += ", \n";
        reply += "NET_CONNECTIONS";
        reply += get_conn_list_string(net_connections);
    } else if (command == "PID") {
        std::ostringstream pid_ss;
        pid_ss << getpid();
        reply += pid_ss.str();
    } else if (command == "STOP") {
        running = false;
        reply = "SUCCESS";
    } else {
       reply = "UNKNOWN";
    } 
    //XXX: possible to get stuck in wrong state here?
    if (zmq_send(control_rep, reply.c_str(), reply.size(), ZMQ_DONTWAIT) < 0) {
        assert(errno & (EAGAIN | EINTR));
    }
}

int main(int argc, char **argv) {
    bool should_daemonize = true;
    std::string vehicle_name(cauv::get_vehicle_name());
    int argii;
    for (argii = 1; argii < argc; argii++) {
        if (!strcmp(argv[argii],"-s")) {
            should_daemonize = false;
        } else {
            vehicle_name = argv[argii];
        }
    }

    std::string ipc_directory(cauv::get_ipc_directory(vehicle_name));
    ipc_directory += "/daemon/";
    create_path(ipc_directory);

    std::string lockfile = ipc_directory + "/lock";
    if (!get_lock_file(lockfile)) {
        fprintf(stderr,"Could not get lock file %s, is another daemon running?\n", lockfile.c_str());
        exit(1);
    }

    if (should_daemonize) {
        daemonize();
        //lock is not preserved across forks
        if(!get_lock_file(lockfile)) {
            exit(1);
        }
    }
    umask(0);

    DaemonContext context(vehicle_name, ipc_directory);
    context.run();

    return 0;
}
