#include <iostream>
#include <ostream>

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/weak_ptr.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/exceptions.hpp>


const static std::size_t Alloc_Size = 0x100000;
const static void* Alloc_Addr = (void*)0x167020000000;
const static char* Alloc_Name = "SMemTest";
const static char* Persistent_Thing_Name = "PersistentThingInstance";
const static char* Persistent_Thing_Handle_Name = "PersistentThingWkPtr";


namespace bip = boost::interprocess;

class SMemTest;

// There's only one instance of SMemTest per-process, use this to get at it.
// Note that the reference count of the returned pointer is 1 -- that is, the
// returned object will be destroyed if the returned pointer is destroyed
// without being copied: it's probably a good idea not to do that too often.
//
// This avoids static-destruction fiascos, since the destructor of this needs
// to do some shared memory (and possibly some network) operations, which may
// not be possible at static destruction.
boost::shared_ptr<SMemTest> getHandle();

typedef bip::string msg_t;
typedef bip::managed_shared_ptr<
            bip::string,
            bip::fixed_managed_shared_memory
        >::type msg_ptr;


const static std::size_t Max_Q_Size = 0x10;
class MsgQ: public boost::noncopyable{
    public:
        MsgQ();
        void pushDiscard(msg_ptr m);
        msg_ptr popWait();

    private:
        bip::interprocess_mutex m_mutex;
        bip::interprocess_condition m_cond_empty;
        std::size_t m_empty_idx;
        msg_ptr m_queue[Max_Q_Size];
};

class PersistentThing: public boost::noncopyable{
    public:
        PersistentThing(bip::fixed_managed_shared_memory const& segment);
        ~PersistentThing();

        // push m onto message queues of all listeners
        void send(msg_ptr m);

        int startReceivingMessages();
        void stopReceivingMessages(int connection_id);

        // block until there is a message available
        msg_ptr receive(int connection_id);


    private:
        typedef MsgQ* msgq_ptr; // because we're using fixed address memory
        typedef std::pair<int, msgq_ptr> connection_value_t;
        typedef bip::allocator<
                    connection_value_t,
                    bip::fixed_managed_shared_memory::segment_manager
                > connection_alloc_t;
        typedef bip::map<int,
                         msgq_ptr,
                         std::less<int>,
                         connection_alloc_t
                > connections_map_t;
        bip::interprocess_mutex m_open_connections_lock;
        connections_map_t m_open_connections;

        bip::interprocess_mutex m_next_connection_id_lock;
        int m_next_connection_id;
};


class SMemTest: public boost::noncopyable{
    public:
        ~SMemTest();
        
        // receive and processes messages for the rest of time
        void receiveMessages();
        
        // send a message
        void send(msg_ptr m);

        // get a handle to the shared memory
        bip::fixed_managed_shared_memory& theMemory();

    private:
        friend boost::shared_ptr<SMemTest> getHandle();

        typedef bip::managed_shared_ptr<
                    PersistentThing,
                    bip::fixed_managed_shared_memory
                >::type persistent_thing_ptr;
        typedef bip::managed_weak_ptr<
                    PersistentThing,
                    bip::fixed_managed_shared_memory
                >::type persistent_thing_wkptr;
        
        SMemTest();

        persistent_thing_ptr findThePersistentThing();
        persistent_thing_ptr getThePersistentThing();
        

        /**** private data ****/
        bip::fixed_managed_shared_memory m_smo; 
        persistent_thing_ptr m_persistent_thing;
};



