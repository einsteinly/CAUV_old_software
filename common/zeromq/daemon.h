#include <string>

namespace cauv {

class ZmqGroupDaemon {
    public:
    ZmqGroupDaemon(const std::string name);
    void test(void);
    void run(void);
    private:

    void handle_control_msg(const std::string msg);
    void send_control_msg(const std::string msg);

    const std::string name;
    bool has_listeners;
    bool has_remote_listeners;
    zmq::context_t zm_context;
    zmq::socket_t zm_pub_sock;
    zmq::socket_t zm_recv_sock;
    zmq::socket_t zm_control_pub;
    zmq::socket_t zm_control_sub;
    zmq::socket_t zm_group_pub;
    zmq::socket_t zm_group_sub;
};

}
