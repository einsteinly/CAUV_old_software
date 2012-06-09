
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>

#include <vector>
#include <map>
#include <set>
#include <utility>
#include <algorithm>

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <xs.h>
#include <assert.h>
#include <stdint.h>
#include <sys/time.h>

#include "daemon_util.h"
#include "zeromq_addresses.h"

typedef std::set<uint32_t> subscriptions_t;
typedef std::map<uint32_t,std::pair <unsigned int, unsigned long int> > stats_t;
typedef std::pair <int, std::string> connection_t;
typedef std::vector< connection_t > conn_list_t;

std::ostream &operator<<(std::ostream &os, const conn_list_t &conn_list) {
    os << "[";
    for(conn_list_t::const_iterator it = conn_list.begin();
        it != conn_list.end(); it++) {

        os << "{" << 
            "\"id\": " << it->first << ", " <<
            "\"string\": " << "\"" << it->second << "\""
            << "}";
        if (it != conn_list.end() - 1) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

std::ostream &operator<<(std::ostream &os, const subscriptions_t &subs) {
    os << "[";
    for(subscriptions_t::const_iterator it = subs.begin();
        it != subs.end(); it++) {
        os << *it << ", ";
    }
    os << "null";
    os << "]";
    return os;
}

std::ostream &operator<<(std::ostream &os, const stats_t &stats) {
    os << "[";
    for(stats_t::const_iterator it = stats.begin();
        it != stats.end(); it++) {

        os << "{ \"id\": " << it->first
           << ", \"messages\": " << it->second.first
           << ", \"bytes\": " << it->second.second
           << "}";
        os << ", ";
    }
    os << "null";
    os << "]";
    return os;
}

class SocketInfo {
    public:
    SocketInfo(void *skt) :
        skt(skt) {};
    void *skt;
    conn_list_t binds;
    conn_list_t connections;
    int connect(std::string connect_str);
    int bind(std::string bind_str);
};

std::ostream &operator<<(std::ostream &os, const SocketInfo &info) {
    os << "{";
    os << "\"connections\" :" << info.connections << ",\n";
    os << "\"binds\" : " << info.binds << "\n";
    os << "}";
    return os;
}

class ConnCmp {
    public:
    ConnCmp(std::string conn_str) :
        conn_str(conn_str) {};
    bool operator() (connection_t other) {
        return other.second == conn_str;
    }
    private:
    std::string conn_str;
};


int SocketInfo::connect(std::string connect_str) {
    if (std::count_if(connections.begin(), connections.end(), ConnCmp(connect_str))) {
        return 0;
    }
    int ret = xs_connect(skt, connect_str.c_str());
    if (ret < 0) {
        return errno;
    } else {
        connections.push_back(connection_t(ret, connect_str));
        return 0;
    }
}

int SocketInfo::bind(std::string bind_str) {
    if (std::count_if(binds.begin(), binds.end(), ConnCmp(bind_str))) {
        return 0;
    }
    int ret = xs_bind(skt, bind_str.c_str());
    if (ret < 0) {
        return errno;
    } else {
        binds.push_back(connection_t(0, bind_str));
        return 0;
    }
}

class XPubSubPair {
    public:
    XPubSubPair(void *ctx);
    ~XPubSubPair(void);
    SocketInfo xsub;
    SocketInfo xpub;
    subscriptions_t subs;
    stats_t stats;

    void pump_message(void);
    void pump_subscription(void);
    private:
    xs_msg_t message_buf;
    XPubSubPair (XPubSubPair &other);
    XPubSubPair &operator=(XPubSubPair &other);
    XPubSubPair *opposite;
};

XPubSubPair::XPubSubPair(void *ctx) :
    xsub(xs_socket(ctx, XS_XSUB)),
    xpub(xs_socket(ctx, XS_XPUB)),
    opposite(NULL) {
    int linger = 300;
    assert(xsub.skt);
    assert(xpub.skt);
    assert(xs_setsockopt(xsub.skt, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_setsockopt(xpub.skt, XS_LINGER, &linger, sizeof(linger)) == 0);
    assert(xs_msg_init(&message_buf) == 0);
}

XPubSubPair::~XPubSubPair(void) {
    if (opposite) {
        opposite->opposite = NULL;
    }
    xs_close(xsub.skt);
    xs_close(xpub.skt);
}

void XPubSubPair::pump_message(void) {
    if (xs_recvmsg(xsub.skt, &message_buf, XS_DONTWAIT) < 0) {
        assert(errno & (EAGAIN | EINTR));
        return;
    }
    if (xs_msg_size(&message_buf) >= sizeof(uint32_t)) {
        const uint32_t msg_id = *reinterpret_cast<const uint32_t*>(xs_msg_data(&message_buf));
        if (msg_id == NODE_MARKER_MSGID) {
            return;
        }
        stats[msg_id].first++;
        stats[msg_id].second += xs_msg_size(&message_buf);
    }
    if (xs_sendmsg(xpub.skt, &message_buf, 0) < 0) {
        assert(errno & (EAGAIN | EINTR));
    }
}

void XPubSubPair::pump_subscription(void) {
    if (xs_recvmsg(xpub.skt, &message_buf, XS_DONTWAIT) < 0) {
        assert(errno & (EAGAIN | EINTR));
        return;
    }
    if (xs_msg_size(&message_buf) >= sizeof(uint32_t) + 1) {
        cauv::subscription_vec_t sub_vec = cauv::parse_subscription_message(
                std::string(reinterpret_cast<char*>(xs_msg_data(&message_buf)), xs_msg_size(&message_buf)));
        const char sub = sub_vec.first; 
        if (sub_vec.second.size() >= 1) {
            const uint32_t msg_id = sub_vec.second[0];
            if (sub_vec.second.size() == 1) {
                if (sub) {
                    subs.insert(msg_id);
                } else {
                    subs.erase(msg_id);
                }
            }
            if (msg_id == NODE_MARKER_MSGID && sub_vec.second.size() == 2) {
                //avoid remote PIDs from appearing as local
                sub_vec.second[1] |= 0x1 << 31;
                std::string sub_buf = cauv::gen_subscription_message(sub_vec);
                xs_send(xsub.skt, sub_buf.c_str(), sub_buf.size(), 0);
                return;
            }
        }
    }
    if (xs_sendmsg(xsub.skt, &message_buf, 0) < 0) {
        assert(errno & (EAGAIN | EINTR));
    }
}

class DaemonContext {
    public:
    DaemonContext(const std::string vehicle_name, const std::string working_directory);
    void run(void);
    ~DaemonContext(void);
    private:
    bool running;
    const std::string working_directory;
    const std::string vehicle_name;

    //unique daemon id
    uint32_t daemon_id;
    void *xs_context;
    void *control_rep;
    //the names refer to the flow of messages. flow of subscriptions is in
    //opposite direction
    XPubSubPair net_to_local;
    XPubSubPair local_to_net;

    void handle_control_message(void);

    DaemonContext &operator=(const DaemonContext &other);
    DaemonContext (const DaemonContext &other);
};

DaemonContext::DaemonContext(const std::string vehicle_name, const std::string working_directory) :
    running(true),
    working_directory(working_directory),
    vehicle_name(vehicle_name),
    xs_context(xs_init()),
    net_to_local(xs_context),
    local_to_net(xs_context) {
    assert(xs_context);

    //seed random number generator
    struct timeval tv;
    assert(gettimeofday(&tv,NULL) == 0);
    srand(getpid() + tv.tv_sec * 1000000 + tv.tv_usec);
    daemon_id = rand();

    int linger = 300;
    control_rep = xs_socket(xs_context,XS_REP);
    assert(control_rep);
    assert(xs_setsockopt(control_rep, XS_LINGER, &linger, sizeof(linger)) == 0);
    std::string control_path = "ipc://" + working_directory + "/control";
    assert(xs_bind(control_rep,control_path.c_str()) >= 0);

    std::string local_sub_path = "ipc://" + working_directory + "/sub";
    assert(local_to_net.xsub.bind(local_sub_path) == 0);
    std::string local_pub_path = "ipc://" + working_directory + "/pub";
    assert(net_to_local.xpub.bind(local_pub_path) == 0);

    cauv::subscription_vec_t sub;
    sub.first = true;
    sub.second.push_back(DAEMON_MARKER_MSGID);
    sub.second.push_back(daemon_id);
    std::string sub_buf = cauv::gen_subscription_message(sub);
    if(xs_send(local_to_net.xsub.skt, sub_buf.c_str(), sub_buf.size(), 0) < 0) {
        printf("%d %s\n",errno,xs_strerror(errno));
        assert(0);
    }
}

DaemonContext::~DaemonContext(void) {
    xs_close(control_rep);
    //xs_term(xs_context);
}

void DaemonContext::run(void) {
    xs_pollitem_t sockets[] = {
        { local_to_net.xsub.skt,
          0, XS_POLLIN, 0
        },
        { local_to_net.xpub.skt,
          0, XS_POLLIN, 0
        },
        { net_to_local.xsub.skt,
          0, XS_POLLIN, 0
        },
        { net_to_local.xpub.skt,
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
                local_to_net.pump_message();
            }
            if (sockets[1].revents & XS_POLLIN) {
                local_to_net.pump_subscription();
            }
            if (sockets[2].revents & XS_POLLIN) {
                net_to_local.pump_message();
            }
            if (sockets[3].revents & XS_POLLIN) {
                net_to_local.pump_subscription();
            }
            if (sockets[4].revents & XS_POLLIN) {
                handle_control_message();
            }
        }
    }
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
    std::ostringstream reply;
    std::string arg;
    ctrl_iss >> command;
    reply << "{";
    std::string error;
    if (command == "CONNECT" || command == "BIND") {
        SocketInfo *socket = NULL;
        std::string socket_name;
        ctrl_iss >> socket_name;
        if (socket_name == "NET_XPUB") {
            socket = &local_to_net.xpub;
        } else if (socket_name == "NET_XSUB") {
            socket = &net_to_local.xsub;
        } else if (socket_name == "LOCAL_XPUB") {
            socket = &net_to_local.xpub;
        } else if (socket_name == "LOCAL_XSUB") {
            socket = &local_to_net.xsub;
        } else {
            reply << "UNKNOWN SOCKET";
        }
        if (socket) {
            std::string specifier; 
            ctrl_iss >> specifier;
            int ret;
            if (command == "CONNECT") {
                ret = socket->connect(specifier);
            } else if (command == "BIND") {
                ret = socket->bind(specifier);
            } else {
                //something wrong with the computer?
                assert(false);
            }
            if (ret) {
                error = xs_strerror(ret);
            }
        }
    } else if (command == "STATS") {
        reply << "\"net_to_local\" : " << net_to_local.stats << ",\n";
        reply << "\"local_to_net\" : " << local_to_net.stats << ",\n";
    } else if (command == "SUBS") {
        reply << "\"net_to_local\" : " << net_to_local.subs << ",\n";
        reply << "\"local_to_net\" : " << local_to_net.subs << ",\n";
    } else if (command == "CONNECTIONS") {
        reply << "\"net_xpub\" : " << local_to_net.xpub << ",\n";
        reply << "\"net_xsub\" : " << net_to_local.xsub << ",\n";
        reply << "\"local_xpub\" : " << net_to_local.xpub << ",\n";
        reply << "\"local_xsub\" : " << local_to_net.xsub << ",\n";
    } else if (command == "PID") {
        reply << "\"pid\" : " << getpid() << ", ";
    } else if (command == "ID") {
        reply << "\"id\" : " << daemon_id << ", ";
    } else if (command == "STOP") {
        running = false;
    } else {
        error = "Unknown command";
    } 
    if (error.size()) {
        reply << "\"success\" : false, ";
        reply << "\"error\" : \"" << error << "\"";
    } else {
        reply << "\"success\" : true";
    }
    reply << "}";
    //XXX: possible to get stuck in wrong state here?
    if (xs_send(control_rep, reply.str().c_str(), reply.str().size(), XS_DONTWAIT) < 0) {
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
