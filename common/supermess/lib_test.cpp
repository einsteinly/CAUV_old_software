/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "shared_mem_test_lib.h"

#include <debug/cauv_debug.h>


int main(int argc, char** argv){
    debug() << "test started...";
    try{
        boost::shared_ptr<SMemTest> p = getHandle();
        boost::shared_ptr<SMemTest> p2 = getHandle();
        boost::shared_ptr<SMemTest> p3 = getHandle();
        boost::shared_ptr<SMemTest> p4 = getHandle();
        assert(p == p2 && p == p3 && p == p4);

        debug() << "sending messages...";
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
            debug() << "waiting for messages...";
            // block receiving other messages (forever)
            p->receiveMessages();
        }

    }catch(std::exception& e){
        error() << "lib_test badness:"
                << e.what();
    }catch(...){
        error() << "lib_test unknown badness";
    }
    debug() << "test complete!";
}

