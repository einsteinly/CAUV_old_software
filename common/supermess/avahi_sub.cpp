#include <string>
#include <vector>

#include <avahi-core/core.h>
#include <avahi-core/lookup.h>
#include <avahi-common/malloc.h>
#include <avahi-common/thread-watch.h>
#include <avahi-common/error.h>

#include <debug/cauv_debug.h>

namespace cauv{


class AvahiSubscriber{
    public:
        typedef std::string address_t;
        typedef std::set<address_t> address_list_t;
        
        AvahiSubscriber(std::string const& type = "_cauv._udp")
            : m_threaded_poll(NULL),
              m_service_browser(NULL),
              m_server(NULL){

            open();
        }
        ~AvahiSubscriber(){
            close();
        }

        void open(){
            int err = 0;
            m_threaded_poll = avahi_threaded_poll_new();
            if(!m_threaded_poll){
                error() << "could not create Avahi threaded_poll";
                return;
            }

            AvahiServerConfig config;
            avahi_server_config_init(&config);
            config.publish_hinfo = 0;
            config.publish_addresses = 0;
            config.publish_workstation = 0;
            config.publish_domain = 0;
            
            // don't enable wide-area stuff
            //avahi_address_parse("192.168.50.1", AVAHI_PROTO_UNSPEC, &m_config.wide_area_servers[0]);
            //config.n_wide_area_servers = 1;
            //config.enable_wide_area = 1;

            m_server = avahi_server_new(
                avahi_threaded_poll_get(m_threaded_poll), &config, NULL, NULL, &err
            );

            avahi_server_config_free(&config);
            
            if(!server){
                error() << "failed to create server:" << avahi_strerror(error);
                close();
                return;
            }

            m_service_browser = avahi_s_service_browser_new(
                m_server,
                AVAHI_IF_UNSPEC,
                AVAHI_PROTO_UNSPEC,
                m_type.c_str(),
                NULL, // domain
                AvahiLookupFlags(0), //flags
                callBrowseCallBack,
                this // userdata (void*)
            );
                

            // spawns new thread
            avahi_threaded_poll_start(m_threaded_poll);
        }

        void close(){
            if(m_threaded_poll)
                avahi_threaded_poll_stop(m_threaded_poll);
            if(m_service_browser)
                avahi_s_service_browser_free(m_service_browser);
            m_service_browser = NULL;
            if(m_server)
                avahi_server_free(m_server);
            m_server = NULL;
            if(m_threaded_poll)
                avahi_threaded_poll_free(m_threaded_poll);
            m_threaded_poll = NULL;
        }

        address_list_t addressesForService(std::string const& cauv_service_type){
        }

    private:
        static void browseCallBack(
            AvahiSServiceBrowser *b,
            AvahiIfIndex interface,
            AvahiProtocol protocol,
            AvahiBrowserEvent event,
            const char *name,   // Service name
            const char *type,   // DNS-SD type, e.g. "_http._tcp"
            const char *domain, // Domain of this service, e.g. "local"
            AvahiLookupResultFlags flags, // Lookup flags 
            void* userdata)
        {
            this_p = (AvahiSubscriber*)userdata;
            debug() << "browseCallback:" << name << type << domain;
            switch(event){
                case AVAHI_BROWSER_FAILURE:
                    error() << "AVAHI_BROWSER_FAILURE";
                    this_p->close();
                    return;
                case AVAHI_BROWSER_NEW:
                    debug() << "AVAHI_BROWSER_NEW";
                    if(!(avahi_s_service_resolver_new(
                             this_p->m_server, interface, protocol, name, type, domain,
                             AVAHI_PROTO_UNSPEC, 0, callResolveCallback, this))){
                        error() << "failed to resolve service" << name << type << domain;
                    }
                case AVAHI_BROWSER_REMOVE:
                    debug() << "AVAHI_BROWSER_REMOVE";
                    break;
                case AVAHI_BROWSER_ALL_FOR_NOW:
                    debug() << "AVAHI_BROWSER_ALL_FOR_NOW";
                    break;
                case AVAHI_BROWSER_CACHE_EXHAUSTED:
                    debug() << "AVAHI_BROWSER_CACHE_EXHAUSTED";
                    break;
            }
        }

        static void resolveCallBack(
            AvahiSServiceResolver *r,
            AvahiIfIndex interface,
            AvahiProtocol protocol,
            AvahiResolverEvent event,
            const char *name,
            const char *type,
            const char *domain,
            const char *host_name,
            const AvahiAddress *address,
            uint16_t port,
            AvahiStringList *txt,
            AvahiLookupResultFlags flags,
            void* userdata)
        {
            debug() << "resolveCallBack:" << name << type << domain;        
            this_p = (AvahiSubscriber*)userdata;
            switch(event){
                case AVAHI_RESOLVER_FAILURE:
                    debug() << "AVAHI_RESOLVER_FAILURE";
                    info() << "failed to resolve service:"
                           << name << type << domain
                           << avahi_strerror(avahi_server_errno(this_p->m_server));
                    return;
                case AVAHI_RESOLVER_FOUND:
                    debug() << "AVAHI_RESOLVER_FOUND";
                    info() "resolved service:" << name << type << domain;

            }
        }
        
        std::map<std::string, address_list_t> m_known_services;
        AvahiThreadedPoll* m_threaded_poll;
        AvahiSServiceBrowser* m_service_browser;
        AvahiServer* m_server;
};

} // namespace cauv

int main(int argc, char** argv){
    
}

