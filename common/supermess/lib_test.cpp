#include "shared_mem_test_lib.h"


int main(int argc, char** argv){
    std::cerr << "test started..." << std::endl;
    try{
        boost::shared_ptr<SMemTest> p = getHandle();
        boost::shared_ptr<SMemTest> p2 = getHandle();
        boost::shared_ptr<SMemTest> p3 = getHandle();
        boost::shared_ptr<SMemTest> p4 = getHandle();
        assert(p == p2 && p == p3 && p == p4);

        std::cerr << "sending messages..." << std::endl;
        for(int i = 1; i < argc; i++){
            // construct unnamed managed shared memory object of msg_t type
            // initialised from argv[i]
            msg_ptr m = bip::make_managed_shared_ptr(
                p->theMemory().construct<msg_t>(bip::anonymous_instance)(argv[i]), p->theMemory()
            );
            
            // send it
            p->send(m);
        }
        
        if(argc == 1){
            std::cerr << "waiting for messages..." << std::endl;
            // block receiving other messages (forever)
            p->receiveMessages();
        }

    }catch(std::exception& e){
        std::cerr << "lib_test badness:" << std::endl;
        std::cerr << e.what() << std::endl;
    }catch(...){
        std::cerr << "lib_test unknown badness" << std::endl;
    }
    std::cerr << "test complete!" << std::endl;
}

