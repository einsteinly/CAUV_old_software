
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <zmq.h>
#include <assert.h>

#include "daemon_util.h"

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

static std::string get_control_multicast(void) {
    return "epgm://" + get_multicast_interface() + ";239.192.1.1:5555";
}

class DaemonContext {
    public:
    DaemonContext(const std::string vehicle_name, const std::string working_directory);
    void run(void);
    ~DaemonContext(void);
    private:
    const std::string working_directory;
    const std::string vehicle_name;
    void *zmq_context;
    void *control_rep;
    void *network_xsub;
    void *network_xpub;
    void *local_xsub;
    void *local_xpub;

    DaemonContext &operator=(const DaemonContext &other);
    DaemonContext (const DaemonContext &other);
};

DaemonContext::DaemonContext(const std::string vehicle_name, const std::string working_directory) :
    working_directory(working_directory),
    vehicle_name(vehicle_name) {
    zmq_context = zmq_ctx_new();
    assert(zmq_context);

    int linger = 0;
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
    assert(zmq_setsockopt(control_req, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(network_xsub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(network_xpub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(local_xsub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    assert(zmq_setsockopt(local_xpub, ZMQ_LINGER, &linger, sizeof(linger)) == 0);
    std::string control_path = "ipc://" + working_directory + "/control";
    assert(zmq_connect(control_rep,control_path.c_str(),control_path.size()) == 0);
}

DaemonContext::~DaemonContext(void) {
    zmq_ctx_destroy(zmq_context);
}

void DaemonContext::run(void) {
    //sleep(10);
}

int main(int argc, char **argv) {
    bool should_daemonize = true;
    std::string vehicle_name("red_herring");
    int argii;
    for (argii = 1; argii < argc; argii++) {
        if (!strcmp(argv[argii],"-s")) {
            should_daemonize = false;
        } else {
            vehicle_name = argv[argii];
        }
    }

    std::string working_directory("/tmp/cauv/");
    working_directory += vehicle_name + "/daemon/";
    create_working_directory(working_directory);

    std::string lockfile = working_directory + "/lock";
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

    DaemonContext context(vehicle_name, working_directory);
    context.run();

    return 0;
}
