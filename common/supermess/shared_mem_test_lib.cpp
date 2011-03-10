#include "shared_mem_test_lib.h"

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>


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
    m_queue[m_empty_idx++ % Max_Q_Size] = m;
    m_cond_empty.notify_one();
}

msg_ptr MsgQ::popWait(){
    bip::scoped_lock<bip::interprocess_mutex> l(m_mutex);
    if(0 == m_empty_idx % Max_Q_Size)
        m_cond_empty.wait(l);
    return m_queue[(--m_empty_idx) % Max_Q_Size];
}



PersistentThing::PersistentThing(bip::fixed_managed_shared_memory const& segment)
    : m_open_connections_lock(),
      m_open_connections(
          std::less<int>(),
          connection_alloc_t(segment.get_segment_manager())
      ),
      m_next_connection_id_lock(),
      m_next_connection_id(1){
    std::cerr << "PersistentThing()" << std::endl;
}

PersistentThing::~PersistentThing(){
    std::cerr << "~PersistentThing" << std::endl;
}

void PersistentThing::send(msg_ptr m){
    bip::scoped_lock<bip::interprocess_mutex> l(m_open_connections_lock);        
    connections_map_t::iterator i;
    std::cerr << "sending: " << *m << std::endl;
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
    bip::scoped_lock<bip::interprocess_mutex> l(m_next_connection_id_lock);
    bip::scoped_lock<bip::interprocess_mutex> m(m_open_connections_lock);

    int connection_id = m_next_connection_id;
    m_next_connection_id++;
    
    // corresponding destroy() in stopReceivingMessages
    msgq_ptr mq = getHandle()->theMemory().construct<MsgQ>(bip::anonymous_instance)();
    m_open_connections[connection_id] = mq;

    return connection_id;
}

void PersistentThing::stopReceivingMessages(int connection_id){
    bip::scoped_lock<bip::interprocess_mutex> m(m_open_connections_lock);
    connections_map_t::iterator i = m_open_connections.find(connection_id);
    getHandle()->theMemory().destroy_ptr(i->second);
    m_open_connections.erase(connection_id);
}

msg_ptr PersistentThing::receive(int connection_id){
    msg_ptr r;
    bip::scoped_lock<bip::interprocess_mutex> l(m_open_connections_lock);            
    connections_map_t::const_iterator i = m_open_connections.find(connection_id);
    l.unlock();
    assert(i != m_open_connections.end());
    r = i->second->popWait();
    return r;
}

SMemTest::~SMemTest(){
    std::cerr << "~SMemTest(): " <<  m_persistent_thing.use_count()-1
              << " other PersistentThing users remain" << std::endl;
}

SMemTest::SMemTest()
    : m_smo(),
      m_persistent_thing(){
    std::cerr << "SMemTest construction..." << std::endl;
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
        std::cerr << "could not attach shared memory: " <<  e.what() << std::endl;
        std::cerr << "cleaning up..." << std::endl;
        boost::interprocess::shared_memory_object::remove(Alloc_Name);
        throw;
    }catch(...){
        std::cerr << "could not attach shared memory" << std::endl;
        std::cerr << "cleaning up..." << std::endl;
        try{
            boost::interprocess::shared_memory_object::remove(Alloc_Name);
        }catch(...){
            std::cerr << "cleanup failed" << std::endl;
        }
        std::cerr << "cleanup complete" << std::endl;
        throw;
    }
    std::cerr << "SMemTest construction complete" << std::endl;
}

// receive and processes messages for the rest of time
void SMemTest::receiveMessages(){
    int connection_id = m_persistent_thing->startReceivingMessages();
    try{
        while(true){
            msg_ptr m = m_persistent_thing->receive(connection_id);
            std::cerr << "id:" << connection_id << ":" << *m << std::endl;
            if(*m == "stop")
                break;
        }
    }catch(...){
        std::cerr << "receiveMessages() ended by exception" << std::endl;
        m_persistent_thing->stopReceivingMessages(connection_id);
        throw;
    }
    std::cerr << "receiveMessages() ended gracefully" << std::endl;
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
            std::cerr << "could not find the PersistentThing" << std::endl;
            throw;
        }
    }catch(...){
        std::cerr << "PersistentThing: fatal?" << std::endl;
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

