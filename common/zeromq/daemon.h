#include <string>

namespace cauv {

class ZmqGroupDaemon {
    public:
    ZmqGroupDaemon(const std::string name);
    void test(void);
    void run(void);
    private:
    const std::string name;
    zmq::context_t zm_context;
    zmq::socket_t zm_pub_sock;
    zmq::socket_t zm_recv_sock;
    zmq::socket_t zm_control_pub;
    zmq::socket_t zm_control_sub;
};

}
