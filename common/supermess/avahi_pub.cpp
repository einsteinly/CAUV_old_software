#include <avahi-client/client.h>
#include <avahi-client/publish.h>
#include <avahi-common/malloc.h>
#include <avahi-common/thread-watch.h>
#include <avahi-common/error.h>

#include <debug/cauv_debug.h>

namespace cauv{

class AvahiPublisher{
    public:
        AvahiPublisher(std::string const& name="CAUVNode",
                    std::string const& type="_cauv._udp",
                    unsigned port=16707)
            : m_threaded_poll(NULL),
              m_entry_group(NULL),
              m_client(NULL),
              m_name(NULL),
              m_type(NULL),
              m_port(0){
            open(name, type, port);
        }

        ~AvahiPublisher(){
            close();
        }

        void open(std::string const& name, std::string const& type, unsigned port){
            close();
            int e = 0;
            m_port = port;
            m_threaded_poll = avahi_threaded_poll_new();
            if(!m_threaded_poll){
                error() << "could not create Avahi threaded_poll";
                return;
            }
            m_name = avahi_strdup(name.c_str());
            m_type = avahi_strdup(type.c_str());
            m_client = avahi_client_new(
                avahi_threaded_poll_get(m_threaded_poll),
                AVAHI_CLIENT_NO_FAIL,
                &callClientCallBack,
                this,
                &e
            );
            if(!m_client){
                error() << "failed to create client:" << avahi_strerror(e);
                close();
                return;
            }
            // spawns new thread
            avahi_threaded_poll_start(m_threaded_poll);
        }

        /*void fooModifiesSomething(){
            // First, block the event loop
            avahi_threaded_poll_lock(m_threaded_poll);

            // Then, do your stuff
            //if (!avahi_service_browser_new(m_client, ....)) {
            //    
            //}

            // Finally, unblock the event loop
            avahi_threaded_poll_unlock(m_threaded_poll);
        }*/

        void close(){
            if(m_threaded_poll)
                avahi_threaded_poll_stop(m_threaded_poll);
            if(m_entry_group)
                avahi_entry_group_free(m_entry_group);
            m_entry_group = NULL;
            if(m_client)
                avahi_client_free(m_client);
            m_client = NULL;
            if(m_threaded_poll)
                avahi_threaded_poll_free(m_threaded_poll);
            m_threaded_poll = NULL;
            if(m_name)
                avahi_free(m_name);
            if(m_type)
                avahi_free(m_type);
            m_name = NULL;
        }

    private:
        static void callClientCallBack(AvahiClient* c,
                                       AvahiClientState s,
                                       void *user_data){
            ((AvahiPublisher*)user_data)->clientCallBack(c, s);
        }

        void clientCallBack(AvahiClient* c, AvahiClientState s){
            // use c rather than m_client: this may be called during creation
            // of m_client
            m_client = c;
            switch(s){
                case AVAHI_CLIENT_S_RUNNING:
                    debug() << "AVAHI_CLIENT_S_RUNNING";
                    createService();
                    break;
                case AVAHI_CLIENT_FAILURE:
                    debug() << "AVAHI_CLIENT_FAILURE";
                    error() << "Avahi client failure"
                            << avahi_strerror(avahi_client_errno(m_client));
                    close();
                    break;
                case AVAHI_CLIENT_S_COLLISION:
                    debug() << "AVAHI_CLIENT_S_COLLISION";
                    warning() << "collision? not sure what to do....";
                    // fall through
                case AVAHI_CLIENT_S_REGISTERING:
                    debug() << "AVAHI_CLIENT_S_REGISTERING";
                    if(m_entry_group)
                        avahi_entry_group_reset(m_entry_group);
                    break;
                case AVAHI_CLIENT_CONNECTING:
                    debug() << "AVAHI_CLIENT_CONNECTING";
            }
        }

        static void callEntryGroupCallback(AvahiEntryGroup* g,
                                           AvahiEntryGroupState s,
                                           void *user_data){
            ((AvahiPublisher*)user_data)->entryGroupCallBack(g, s);
        }

        void entryGroupCallBack(AvahiEntryGroup* g, AvahiEntryGroupState s){
            // m_entry_group may not equal g yet,  because this callback can be
            // called during creation
            m_entry_group = g;
            switch(s){
                case AVAHI_ENTRY_GROUP_ESTABLISHED:
                    debug() << "AVAHI_ENTRY_GROUP_ESTABLISHED";
                    info() << "successfully established" << m_name << "service";
                    break;
                case AVAHI_ENTRY_GROUP_COLLISION:
                    debug() << "AVAHI_ENTRY_GROUP_COLLISION";
                    error() << "avahi_entry_group collision: not sure what to do?";
                    break;
                case AVAHI_ENTRY_GROUP_FAILURE:
                    debug() << "AVAHI_ENTRY_GROUP_FAILURE";
                    error() << "avahi_entry_group failure";
                    close();
                    break;
                case AVAHI_ENTRY_GROUP_UNCOMMITED:
                case AVAHI_ENTRY_GROUP_REGISTERING:
                    break;
            }
        }

        void createService(){
            int err = 0;
            if(!m_entry_group)
                m_entry_group = avahi_entry_group_new(m_client, callEntryGroupCallback, this);
            if(!m_entry_group){
                error() << "createService() could not create entry group";
                close();
            }
            
            if(avahi_entry_group_is_empty(m_entry_group)){
                debug() << "Adding service" << m_name;
                // this is the bit where we actually publish a service:
                err = avahi_entry_group_add_service(
                    m_entry_group,
                    AVAHI_IF_UNSPEC, // all interfaces
                    AVAHI_PROTO_UNSPEC, // all protocols
                    AvahiPublishFlags(0), // flags, "Usually 0, unless you know what you do"
                    m_name, // Must be valid service name: < 63 characters valid UTF-8. not NULL
                    m_type, // Service type, such as _http._tcp. not NULL
                    NULL,   // Domain, NULL recommended (let daemon decide)
                    NULL,   // Host, NULL recommended (use local hostname)
                    m_port,  // IP port of service
                    "monkey=balls", // record string
                    "redherring=absolutecarp", // record string
                    NULL    // list of parameters must be NULL-terminated
                );
                if(err)
                    error() << "failed to add service:" << avahi_strerror(err);

                if(((err = avahi_entry_group_commit(m_entry_group)) < 0)){
                    error() << "Failed to commit group:" << avahi_strerror(err);
                    close();
                }
            }
        }
        
        AvahiThreadedPoll* m_threaded_poll;
        AvahiEntryGroup* m_entry_group;
        AvahiClient* m_client;
        char* m_name;
        char* m_type;
        unsigned m_port;
};

} // namespace cauv


int main(int argc, char** argv){
    cauv::AvahiPublisher pub("CAUVNode", "_cauv._udp", 16707);
    debug() << "setup complete, sleeping...";
    sleep(30);
}

