#include "shared_mem_test_lib.h"

#include <debug/cauv_debug.h>

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>

boost::shared_ptr<SMemTest> getHandle(){
    static boost::weak_ptr<SMemTest> handle;
    
    boost::shared_ptr<SMemTest> r = handle.lock();
    if(!r){
        r = boost::shared_ptr<SMemTest>(new SMemTest);
        handle = r;
    }

    return r;
}


MsgQ::MsgQ()
    : m_empty_idx(0){
}
void MsgQ::pushDiscard(msg_ptr m){
    bip::scoped_lock<bip::interprocess_mutex> l(m_mutex);
    if(m_empty_idx % Max_Q_Size == Max_Q_Size-1)
        warning() << "queue full, messages may be discarded";
    m_queue[m_empty_idx++ % Max_Q_Size] = m;
    m_cond_empty.notify_one();
}

void MsgQ::pushWait(msg_ptr m){
    bip::scoped_lock<bip::interprocess_mutex> l(m_mutex);
    while(m_empty_idx % Max_Q_Size == Max_Q_Size-1)
        m_cond_full.wait(l);
    m_queue[m_empty_idx++ % Max_Q_Size] = m;
    m_cond_empty.notify_one();
}

msg_ptr MsgQ::popWait(){
    bip::scoped_lock<bip::interprocess_mutex> l(m_mutex);
    while(0 == m_empty_idx % Max_Q_Size)
        m_cond_empty.wait(l);
    static int dropped = 0;
    int dropped_more = 0;
    if((dropped_more = m_empty_idx - (m_empty_idx % Max_Q_Size) - dropped)){
        dropped += dropped_more;
        warning() << "dropped" << dropped_more << "messages:"
                  << dropped << "dropped in total";
    }
    msg_ptr r = m_queue[(--m_empty_idx) % Max_Q_Size];
    m_queue[m_empty_idx % Max_Q_Size].reset();
    m_cond_full.notify_one();
    return r;
}

void MsgQ::clear(){
    bip::scoped_lock<bip::interprocess_mutex> l(m_mutex);
    m_empty_idx = 0;
    // make sure destructors get called on anything left in queue
    for(std::size_t i = 0; i < Max_Q_Size; i++)
        m_queue[i].reset();
}

PersistentThing::PersistentThing(bip::fixed_managed_shared_memory const& segment)
    : m_open_connections_lock(),
      m_open_connections(
          std::less<int>(),
          connection_alloc_t(segment.get_segment_manager())
      ),
      m_next_connection_id_lock(),
      m_next_connection_id(1){
    debug() << "PersistentThing()";
}

PersistentThing::~PersistentThing(){
    debug() << "~PersistentThing";
}

void PersistentThing::send(msg_ptr m){
    bip::sharable_lock<mutex_t> l(m_open_connections_lock);        
    connections_map_t::iterator i;
    debug() << "sending:" << *m;
    for(i = m_open_connections.begin(); i != m_open_connections.end(); i++)
        // all this error handling is largely pointless... if something
        // connected to the shared memory dies abruptly then the managed shared
        // memory structures just get b0rked
        //try{
        //    if(i->second)
                i->second->pushDiscard(m);
        //}catch(std::exception& e){
        //    std::cerr << "sending failed: " << e.what() << std::endl;
        //    std::cerr << "recipient will be ignored from now own" << std::endl;
        //    i->second = NULL;
        //}
    
}

int PersistentThing::startReceivingMessages(){
    bip::scoped_lock<mutex_t> l(m_next_connection_id_lock);
    bip::scoped_lock<mutex_t> m(m_open_connections_lock);

    int connection_id = m_next_connection_id;
    m_next_connection_id++;
    
    // corresponding destroy() in stopReceivingMessages
    msgq_ptr mq = getHandle()->theMemory().construct<MsgQ>(bip::anonymous_instance)();
    m_open_connections[connection_id] = mq;

    return connection_id;
}

void PersistentThing::stopReceivingMessages(int connection_id){
    bip::scoped_lock<mutex_t> m(m_open_connections_lock);
    connections_map_t::iterator i = m_open_connections.find(connection_id);
    getHandle()->theMemory().destroy_ptr(i->second);
    m_open_connections.erase(connection_id);
}

void PersistentThing::clearQueues(){
    bip::scoped_lock<mutex_t> m(m_open_connections_lock);
    connections_map_t::iterator i;
    for(i = m_open_connections.begin(); i != m_open_connections.end(); i++)
        i->second->clear();
}

msg_ptr PersistentThing::receive(int connection_id){
    msg_ptr r;
    bip::sharable_lock<mutex_t> l(m_open_connections_lock);            
    connections_map_t::const_iterator i = m_open_connections.find(connection_id);
    l.unlock();
    assert(i != m_open_connections.end());
    r = i->second->popWait();
    return r;
}

SMemTest::~SMemTest(){
    debug() << "~SMemTest(): " <<  m_persistent_thing.use_count()-1
            << " other PersistentThing users remain";
    warning() << "clearing message queues on program exit: trying to avoid crashiness";
    m_persistent_thing->clearQueues();
}

SMemTest::SMemTest()
    : m_smo(),
      m_persistent_thing(){
    debug() << "SMemTest construction...";
    try{
        m_smo = bip::fixed_managed_shared_memory(
            bip::open_or_create,
            Alloc_Name,
            Alloc_Size,
            Alloc_Addr
        );

        /*size_t sm_size = m_smo.get_segment_manager()->get_size();
        if(sm_size != Alloc_Size){
            std::cerr << "shared memory allocation failed: "
                      << sm_size << " / " << Alloc_Size << std::endl;
            //std::cerr <<  m_smo.get_segment_manager()->get_min_size() << std::endl;
            //throw std::runtime_error("badness");
        }*/

        m_persistent_thing = getThePersistentThing();

    }catch(bip::interprocess_exception& e){
        error() << "could not attach shared memory: " <<  e.what() << std::endl
                << "cleaning up...";
        boost::interprocess::shared_memory_object::remove(Alloc_Name);
        throw;
    }catch(...){
        error() << "could not attach shared memory" << std::endl
                << "cleaning up...";
        try{
            boost::interprocess::shared_memory_object::remove(Alloc_Name);
        }catch(...){
            error() << "cleanup failed";
        }
        info() << "cleanup complete";
        throw;
    }
    debug() << "SMemTest construction complete";
}

// receive and processes messages for the rest of time
void SMemTest::receiveMessages(){
    int connection_id = m_persistent_thing->startReceivingMessages();
    try{
        while(true){
            msg_ptr m = m_persistent_thing->receive(connection_id);
            debug() << "id:" << connection_id << ":" << *m;
            if(*m == "stop")
                break;
        }
    }catch(...){
        error() << "receiveMessages() ended by exception";
        m_persistent_thing->stopReceivingMessages(connection_id);
        throw;
    }
    debug() << "receiveMessages() ended gracefully";
    m_persistent_thing->stopReceivingMessages(connection_id);            
}

void SMemTest::send(msg_ptr m){
    m_persistent_thing->send(m);
}

bip::fixed_managed_shared_memory& SMemTest::theMemory(){
    return m_smo;
}

SMemTest::persistent_thing_ptr SMemTest::findThePersistentThing(){
    persistent_thing_wkptr t = *m_smo.find<persistent_thing_wkptr>(
        Persistent_Thing_Handle_Name
    ).first;
    persistent_thing_ptr r = t.lock();
    return r;
}

SMemTest::persistent_thing_ptr SMemTest::getThePersistentThing(){
    persistent_thing_ptr r;
    try{
        r = bip::make_managed_shared_ptr(
            m_smo.construct<PersistentThing>(Persistent_Thing_Name)(m_smo),
            m_smo
        );
        m_smo.construct<persistent_thing_wkptr>(Persistent_Thing_Handle_Name)(r);
    }catch(bip::interprocess_exception& e){
        // must already exist
        r = findThePersistentThing();
        if(!r){
            error() << "could not find the PersistentThing";
            throw;
        }
    }catch(...){
        error() << "PersistentThing: fatal?";
        throw;
    }
    /*
    persistent_thing_ptr check = findThePersistentThing();
    if(!check || check != r){
        std::cerr << "PerstistentThing find check failed" << std::endl;
        throw std::runtime_error("badness");
    }*/

    return r;
}

