
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
#include <xs.h>
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
    void *xs_context;
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
    xs_msg_t message_buf;

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
    xs_context = xs_init();
    assert(xs_context);

    int linger = 300;
    control_rep = xs_socket(xs_context,XS_REP);
    network_xsub = xs_socket(xs_context,XS_XSUB);
    network_xpub = xs_socket(xs_context,XS_XPUB);
    local_xsub = xs_socket(xs_context,XS_XSUB);
    local_xpub = xs_socket(xs_context,XS_XPUB);
    assert(control_rep);
    assert(network_xsub);
    assert(network_xpub);
    assert(local_xsub);
    assert(local_xpub);
    assert(xs_setsockopt(control_rep, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_setsockopt(network_xsub, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_setsockopt(network_xpub, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_setsockopt(local_xsub, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_setsockopt(local_xpub, XS_LINGER, &linger, sizeof(linger)) == 0);
    std::string control_path = "ipc://" + working_directory + "/control";
    assert(xs_bind(control_rep,control_path.c_str()) == 0);
    std::string local_sub_path = "ipc://" + working_directory + "/sub";
    assert(xs_bind(local_xsub, local_sub_path.c_str()) == 0);

    assert(xs_msg_init(&message_buf) == 0);
    std::string sub_buf;
    uint32_t msgid = DAEMON_MARKER_MSGID;
    sub_buf += "\1";
    sub_buf += std::string(reinterpret_cast<char*>(&msgid),sizeof(msgid));
    assert(xs_send(local_xsub, sub_buf.c_str(), sub_buf.size(), 0) >= 0);
}

DaemonContext::~DaemonContext(void) {
    xs_close(control_rep);
    xs_close(network_xsub);
    xs_close(network_xpub);
    xs_close(local_xsub);
    xs_close(local_xpub);
    xs_term(xs_context);
}

void DaemonContext::pump_message(void *from, void *to, stats_t *stats, subscriptions_t *subs) {
    int flags = 0;
    if (xs_recvmsg(from, &message_buf, XS_DONTWAIT) < 0) {
        assert(errno & (EAGAIN | EINTR));
    } else {
        if (stats) {
            if (xs_msg_size(&message_buf) >= sizeof(uint32_t)) {
                const uint32_t msg_id = *reinterpret_cast<const uint32_t*>(xs_msg_data(&message_buf));
                if (msg_id == NODE_MARKER_MSGID) {
                    return;
                }
                (*stats)[msg_id].first++;
                (*stats)[msg_id].second += xs_msg_size(&message_buf);
            }
        }
        if (subs) {
            if (xs_msg_size(&message_buf) >= sizeof(uint32_t) + 1) {
                const char sub = *reinterpret_cast<const char*>(xs_msg_data(&message_buf));
                const uint32_t msg_id = *reinterpret_cast<const uint32_t*>(
                                            reinterpret_cast<const char*>(xs_msg_data(&message_buf)) + 1);
                if (sub) {
                    subs->insert(msg_id);
                } else {
                    subs->erase(msg_id);
                }
            }
        }

        if (xs_sendmsg(to, &message_buf, flags) < 0) {
            assert(errno & (EAGAIN | EINTR));
        }
    } 
}

void DaemonContext::run(void) {
    xs_pollitem_t sockets[] = {
        { local_xsub,
          0, XS_POLLIN, 0
        },
        { local_xpub,
          0, XS_POLLIN, 0
        },
        { network_xsub,
          0, XS_POLLIN, 0
        },
        { network_xpub,
          0, XS_POLLIN, 0
        },
        { control_rep,
          0, XS_POLLIN, 0
        }
    };
    while(running) {
        int rc = xs_poll(sockets, sizeof(sockets)/sizeof(sockets[0]), 10000);
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
            if (sockets[0].revents & XS_POLLIN) {
                pump_message(local_xsub, network_xpub, &stats_local_to_net, NULL);
            }
            if (sockets[1].revents & XS_POLLIN) {
                pump_message(local_xpub, network_xsub, NULL, &subs_net_to_local);
            }
            if (sockets[2].revents & XS_POLLIN) {
                pump_message(network_xsub, local_xpub, &stats_net_to_local, NULL);
            }
            if (sockets[3].revents & XS_POLLIN) {
                pump_message(network_xpub, local_xsub, NULL, &subs_local_to_net);
            }
            if (sockets[4].revents & XS_POLLIN) {
                handle_control_message();
            }
        }
    }
}

std::string DaemonContext::handle_connect_message(void *socket, std::string connection, conn_list_t &conn_list) {
    std::string reply;
    if (xs_connect(socket, connection.c_str()) < 0) {
        reply = "FAILED ";
        reply += xs_strerror(errno);
    } else {
        reply = "SUCCESS";
        conn_list.push_back(connection);
    }
    return reply;
}

std::string DaemonContext::handle_bind_message(void *socket, std::string bind, conn_list_t &conn_list) {
    std::string reply;
    if (xs_bind(socket, bind.c_str()) < 0) {
        reply = "FAILED ";
        reply += xs_strerror(errno);
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
    xs_msg_t message;
    assert(xs_msg_init(&message) == 0);
    if (xs_recvmsg(control_rep, &message, XS_DONTWAIT) < 0) {
        if (errno & EFSM) {
            //try to clear state
            xs_send(control_rep, "", 0, XS_DONTWAIT);
        } else {
            assert(errno & (EAGAIN | EINTR));
        }
        return;
    }
    std::string control_message(reinterpret_cast<char*> (xs_msg_data(&message)),
                                xs_msg_size(&message));
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
    if (xs_send(control_rep, reply.c_str(), reply.size(), XS_DONTWAIT) < 0) {
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
