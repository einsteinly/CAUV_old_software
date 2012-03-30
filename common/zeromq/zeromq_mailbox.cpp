#include "zeromq_mailbox.h"
#include "zeromq_addresses.h"

#include <unistd.h>
#include <dirent.h>
#include <signal.h>
#include <sys/stat.h>
#include <time.h>

#include <sstream>
#include <algorithm>

#include <debug/cauv_debug.h>

#include <generated/types/message.h>
#include <generated/groupmap.h>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/locks.hpp>

namespace cauv {

ZeroMQMailbox::ZeroMQMailbox(std::string name) :
    name(name),
    zm_context(),
    pub(zm_context, XS_XPUB),
    sub(zm_context, XS_SUB),
    send_queue_push(zm_context, XS_PUSH),
    send_queue_pull(zm_context, XS_PULL),
    sub_queue_push(zm_context, XS_PUSH),
    sub_queue_pull(zm_context, XS_PULL),
    daemon_control(zm_context, XS_REQ),
    m_monitoring(false),
    m_interrupted(false),
    daemon_connected(false) {


    //internal message queues
    send_queue_pull.bind("inproc://send_queue");
    send_queue_push.connect("inproc://send_queue");

    sub_queue_pull.bind("inproc://sub_queue");
    sub_queue_push.connect("inproc://sub_queue");

    std::string ipc_directory = get_ipc_directory();

    std::ostringstream sub_bind_ss;
    mode_t old_umask = umask(0);

    sub_bind_ss << "ipc://" << ipc_directory << "/" << name << "." << getpid();
    sub_bind = sub_bind_ss.str();
    sub.bind(sub_bind.c_str());

    uint32_t marker_sub[2];
    marker_sub[0] = NODE_MARKER_MSGID;
    marker_sub[1] = getpid();
    sub.setsockopt(XS_SUBSCRIBE, marker_sub, sizeof(marker_sub));

    std::string daemon_control_str;
    daemon_control_str = "ipc://" + ipc_directory + "/daemon/control";
    daemon_control.connect(daemon_control_str.c_str());

    std::string daemon_connect_cmd;
    daemon_connect_cmd = "LOCAL_CONNECT " + sub_bind; 
    daemon_control.send(daemon_connect_cmd.c_str(),daemon_connect_cmd.size());

    send_connect_pids = scan_ipc_dir(ipc_directory);

    std::string daemon_sub_str;
    daemon_sub_str = "ipc://" + ipc_directory + "/daemon/sub";
    pub.connect(daemon_sub_str.c_str());

    umask(old_umask);
}

static void delete_message_shared_ptr(void *data, void *hint) {
#ifdef CAUV_DEBUG_MESSAGES
    debug(7) << "deleting message shared pointer";
#endif
    boost::shared_ptr<const Message> *msg = 
        reinterpret_cast<boost::shared_ptr<const Message>*>(hint);
    delete msg;
}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message,
                                       MessageReliability reliability) {
    if(m_interrupted) {
        return 0;
    }
    boost::lock_guard<boost::mutex> lock(m_send_mutex);
    void *msg_shared_ptr = new boost::shared_ptr<const Message>(message);
    xs::message_t message_wrap(msg_shared_ptr, sizeof(boost::shared_ptr<const Message>),
                                delete_message_shared_ptr, msg_shared_ptr);
    send_queue_push.send(message_wrap);
    //doesn't actually report bytes written, since we don't know at this stage
    return 1;
}

int ZeroMQMailbox::sendMessage(boost::shared_ptr<const Message> message,
                                       MessageReliability reliability, const std::string&) {
    return sendMessage(message, reliability);
}

void ZeroMQMailbox::joinGroup(const std::string& groupName) {
    boost::lock_guard<boost::mutex> lock(m_sub_mutex);
    SubscriptionMessage submsg;
    submsg.subscribe = true;
    BOOST_FOREACH(uint32_t id, get_ids_for_group(groupName)) {
        submsg.msg_id = id;
        sub_queue_push.send(&submsg, sizeof(submsg));
    }
}

void ZeroMQMailbox::leaveGroup(const std::string& groupName) {
    boost::lock_guard<boost::mutex> lock(m_sub_mutex);
    SubscriptionMessage submsg;
    submsg.subscribe = true;
    BOOST_FOREACH(uint32_t id, get_ids_for_group(groupName)) {
        submsg.msg_id = id;
        sub_queue_push.send(&submsg, sizeof(submsg));
    }
}

void ZeroMQMailbox::subMessage(const Message &msg) {
    boost::lock_guard<boost::mutex> lock(m_sub_mutex);
    SubscriptionMessage submsg;
    submsg.subscribe = true;
    submsg.msg_id = msg.id();
    sub_queue_push.send(&submsg, sizeof(submsg));
}

void ZeroMQMailbox::unSubMessage(const Message &msg) {
    boost::lock_guard<boost::mutex> lock(m_sub_mutex);
    SubscriptionMessage submsg;
    submsg.subscribe = false;
    submsg.msg_id = msg.id();
    sub_queue_push.send(&submsg, sizeof(submsg));
}

void ZeroMQMailbox::startMonitoringAsync(void) {
    m_monitoring = true;
    m_interrupted = false;
    m_thread = boost::thread(&ZeroMQMailbox::doMonitoring,this);
}

void ZeroMQMailbox::startMonitoringSync(void) {
    m_monitoring = true;
    m_interrupted = false;
    doMonitoring();
}

void ZeroMQMailbox::stopMonitoring(void) {
    m_interrupted = true;
    if (m_thread.get_id() != boost::thread::id()) {
        m_thread.join();
    }
}

bool ZeroMQMailbox::isMonitoring(void) {
    return m_monitoring;
}

void ZeroMQMailbox::addMessageObserver(boost::shared_ptr<MessageObserver> observer) {
    addObserver(observer);
}

void ZeroMQMailbox::removeMessageObserver(boost::shared_ptr<MessageObserver> observer) {
    removeObserver(observer);
}

void ZeroMQMailbox::clearMessageObservers(void) {
    clearObservers();
}

ZeroMQMailbox::pids_t ZeroMQMailbox::scan_ipc_dir(std::string dir) {
    DIR *ipc_dir = opendir(dir.c_str());
    struct dirent *entry;
    pids_t wait_pids;
    while ((entry = readdir(ipc_dir))) {
        if (entry->d_type == DT_SOCK) {
            std::string filename(entry->d_name);
            std::string::iterator it = filename.begin();
            while (std::find(it,filename.end(),'.') != filename.end()) {
                it = std::find(it,filename.end(),'.');
                it++;
            }
            std::string pid_str(it,filename.end());
            uint32_t pid = atoi(pid_str.c_str());
            if (pid == 0) {
                continue;
            }
            if (pid == static_cast<uint32_t>(getpid())) {
                continue;
            }
            if (kill(pid,0) < 0) {
                if (errno == ESRCH) {
                    debug() << "socket" << filename << "appears to be orphaned, trying to unlink";
                    unlink((dir + "/" + filename).c_str());
                    continue;
                }
            }
            
            wait_pids.insert(pid);
            std::string pub_connect = "ipc://" + dir + "/" + filename;
            debug(2) << "connecting to" << pub_connect;
            pub.connect(pub_connect.c_str());
            connections.insert(pub_connect);
        }
    }
    closedir(ipc_dir);
    return wait_pids;
}


void ZeroMQMailbox::send_connect_message (uint32_t pid) {
    std::string buf;
    uint32_t sub_id = NODE_MARKER_MSGID;
    buf += std::string(reinterpret_cast<char*>(&sub_id), sizeof(sub_id));
    buf += std::string(reinterpret_cast<char*>(&pid), sizeof(sub_id));
    buf += sub_bind;
    pub.send(buf.c_str(),buf.size());
}

void ZeroMQMailbox::handle_pub_message (void) {
    xs::message_t pub_message;
    if (!pub.recv(&pub_message, XS_DONTWAIT)) {
        return;
    }
    char *data = reinterpret_cast<char*>(pub_message.data());
    unsigned int len = pub_message.size();
    if (len < sizeof(uint32_t) + 1) {
        return;
    }

    char is_sub = *data;
    uint32_t sub_id = *reinterpret_cast<uint32_t*>(data + 1); 

    //poor man's deserialisation for pid marker
    if (sub_id == NODE_MARKER_MSGID && len >= sizeof(uint32_t) * 2 + 1) {
        uint32_t pid = *(reinterpret_cast<uint32_t*>(data + 1) + 1);
        debug(3) << "pid" << pid << "connected";
        if (send_connect_pids.count(pid)) {
            debug(3) << "sending connection for pid" << pid;
            send_connect_message(pid);
            send_connect_pids.erase(pid);
        }
    }

    if (sub_id == DAEMON_MARKER_MSGID) {
        debug() << "connected to daemon";
        daemon_connected = true;
        return;
    }

    if (len > sizeof(uint32_t) + 1) {
        return;
    }
    if (is_sub) {
        debug(5) << "remote subscribed to message id" << sub_id;
        publications.insert(sub_id);
    } else {
        debug(5) << "remote unsubscribed from message id" << sub_id;
        publications.erase(sub_id);
    }
}

void ZeroMQMailbox::handle_sub_message(void) {
    xs::message_t inc_message;
    if (!sub.recv(&inc_message, XS_DONTWAIT)) {
        return;
    }
    char *data = reinterpret_cast<char*>(inc_message.data());
    unsigned int len = inc_message.size();
    if (len < sizeof(uint32_t)) {
        return;
    }

    uint32_t msg_id = *reinterpret_cast<uint32_t*>(data);

    if (msg_id == NODE_MARKER_MSGID) {
        uint32_t pid = *(reinterpret_cast<uint32_t*>(data) + 1);
        if (pid != (uint32_t)getpid()) {
            warning() << "received connect message not intended for this pid!";
            warning() << "message intended for pid" << pid;
            return;
        }
        std::string connect_str(data + sizeof(uint32_t) * 2, inc_message.size() - sizeof(uint32_t) * 2);
        debug(3) << "connecting to" << connect_str;
        pub.connect(connect_str.c_str());
        connections.insert(connect_str);
        return;
    }

    byte *dataptr = reinterpret_cast<byte*>(inc_message.data());
    //!!! Seems like the copy involved here could be avoided somehow...
    notifyObservers(boost::make_shared<const svec_t>(dataptr,dataptr + inc_message.size()));
}

static void delete_message_bytes(void *data, void *hint) {
#ifdef CAUV_DEBUG_MESSAGE
    debug(9) << "deleting message bytes shared pointer";
#endif
    const_svec_ptr *bytes = reinterpret_cast<const_svec_ptr*>(hint);
    delete bytes;
}

void ZeroMQMailbox::handle_send_message(void) {
    xs::message_t ptr_message;
    send_queue_pull.recv(&ptr_message);
    boost::shared_ptr<const Message> *msg_ptr = reinterpret_cast<boost::shared_ptr<const Message>*>(ptr_message.data());
    if (!publications.count((*msg_ptr)->id())) {
        return;
    }
    const_svec_ptr bytes = (*msg_ptr)->toBytes();
    void *bytes_shared = reinterpret_cast<void*> (new const_svec_ptr(bytes));

    //!!! ugly cast to remove constness
    xs::message_t msg(const_cast<void*>(reinterpret_cast<const void*>(bytes->data())),
                       bytes->size(), delete_message_bytes, bytes_shared);
#ifdef CAUV_DEBUG_MESSAGES
    debug(9) << "sending message";
#endif
    pub.send(msg);
}

void ZeroMQMailbox::handle_subscription_message(void) {
    SubscriptionMessage sub_msg;
    sub_queue_pull.recv(&sub_msg, sizeof(sub_msg));
    if (sub_msg.subscribe) {
        debug(2) << "subscribing to message id" << sub_msg.msg_id;
        if (subscriptions[sub_msg.msg_id] == 0) {
            sub.setsockopt(XS_SUBSCRIBE,&sub_msg.msg_id, sizeof(sub_msg.msg_id));
        }
        subscriptions[sub_msg.msg_id]++;
    } else {
        debug(2) << "unsubscribing from message id" << sub_msg.msg_id;
        if (subscriptions[sub_msg.msg_id] == 0) {
            warning() << "unsubscribed too many times from id" << sub_msg.msg_id;
        } else {
            subscriptions[sub_msg.msg_id]--;
        }
        if (subscriptions[sub_msg.msg_id] == 0) {
            sub.setsockopt(XS_UNSUBSCRIBE,&sub_msg.msg_id, sizeof(sub_msg.msg_id));
        }
    }
}

void ZeroMQMailbox::handle_daemon_message(void) {
    xs::message_t reply_msg;
    daemon_control.recv(&reply_msg);
    std::string reply(reinterpret_cast<char*>(reply_msg.data()), reply_msg.size());
    if (reply != "SUCCESS") {
        warning() << "got unexpected reply from daemon:" << reply;
    }
}

void ZeroMQMailbox::doMonitoring(void) {
    xs::pollitem_t sockets[] = {
        { pub,
          0, XS_POLLIN, 0
        },
        { sub,
          0, XS_POLLIN, 0
        },
        { send_queue_pull,
          0, 0, 0
        },
        { sub_queue_pull,
          0, XS_POLLIN, 0
        },
        { daemon_control,
          0, XS_POLLIN, 0
        }
    };
    bool starting_up = true;
    int timeout = 50;
    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    while (!m_interrupted || starting_up) {
        //use C API for this part because it's simpler
        int rc = xs_poll(sockets, sizeof(sockets)/sizeof(xs::pollitem_t), timeout);
        if (starting_up) {
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            int diff = (current_time.tv_sec - start_time.tv_sec) * 1000;
            diff += (current_time.tv_nsec - start_time.tv_nsec) / 1000000L;
            if ((send_connect_pids.empty() && daemon_connected) || diff > 200) {
                if (!send_connect_pids.empty()) {
                    warning() << "some sockets have not connected yet, starting up anyway";
                }
                if (!daemon_connected) {
                    warning() << "Daemon not connected yet, starting up anyway";
                }
                starting_up = false;
                timeout = 300;
                sockets[2].events |= XS_POLLIN;
                debug() << "mailbox started up, sending messages now";
            }
        }
        if (rc < 0) {
            switch(errno) {
            case ETERM:
                error() << "Socket got closed unexpectedly!. Stopping monitoring";
                m_monitoring = false;
                break;
            case EAGAIN:
            case EINTR:
                break;
            case EFAULT:
            default:
                error() << "error calling xs_poll, Stopping monitoring";
                m_monitoring = false;
            }
            continue;
        } 
        //handle messages in priority order of:
        // publication messages
        // subscription messages
        // received messages
        // sending messages
        //
        //This gives the greatest chance that all is in order before sending
        //messages
        if (sockets[0].revents & XS_POLLIN) {
            handle_pub_message();
        } else if (sockets[3].revents & XS_POLLIN) {
            handle_subscription_message();
        } else if (sockets[1].revents & XS_POLLIN) {
            handle_sub_message();
        } else if (sockets[2].revents & XS_POLLIN) {
            handle_send_message();
        }

        if (sockets[4].revents & XS_POLLIN) {
            handle_daemon_message();
        }
    }
    debug() << "Finishing sending messages";
    xs::pollitem_t send_poll = {
        send_queue_pull,
        0, XS_POLLIN, 0 };
    while (true) {
        int rc = xs_poll(&send_poll,1,100);
        if (rc <= 0) {
            break;
        } else {
            handle_send_message();
        }
    }
    m_monitoring = false;
    debug() << "Mailbox stopped monitoring";
}

}
