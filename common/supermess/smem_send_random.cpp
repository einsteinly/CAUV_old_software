#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>

#include <utility/string.h>
#include <utility/performance.h>

#include <debug/cauv_debug.h>

#include "shared_mem_test_lib.h"

using std::size_t;

static size_t Size_Table_Max_Size = 1000;

int main(int argc, char** argv){
    if(argc <  3 || argc > 4){
        error() << "usage:\n" << argv[0] << " SIZE REPEAT (SIZE_DEVIATION)";
        return 1;
    }
    
    size_t size = fromStr<size_t>(argv[1]);
    size_t repeat = fromStr<size_t>(argv[2]);
    size_t size_dev = 0;
    if(argc == 4)
        size_dev = fromStr<size_t>(argv[3]);
    
    if(size > Alloc_Size || size == 0){
        error() << "SIZE should be between 1 and " << Alloc_Size;
        return 1;
    }
    if(size_dev >= size - Alloc_Size){
        error() << "SIZE_DEVIATION should be < SIZE - " << Alloc_Size;
        return 1;
    }
    
    std::vector<size_t> sizes(std::min(repeat, Size_Table_Max_Size), 0);
    for(std::vector<size_t>::iterator i = sizes.begin(); i != sizes.end(); i++){
        *i = size;
        if(size_dev)
            *i += std::rand() % size_dev;
    }

    try{
        boost::shared_ptr<SMemTest> p = getHandle();
        
        Timer t;
        t.start();
        for(size_t i = 0; i < repeat; i++){
            // construct unnamed managed shared memory object of msg_t type,
            // containing sizes[i] % Size_Table_Max_Size
            const size_t size_to_send = sizes[i % Size_Table_Max_Size];
            //std::cout << "size to send = " << size_to_send << std::endl;
            msg_ptr m = bip::make_managed_shared_ptr(
                p->theMemory().construct<msg_t>(bip::anonymous_instance)(size_to_send, char('a' + i % 26)), p->theMemory()
            );
            // fill what we're going to send with something fun
            assert(m->size() == size_to_send);
            
            //std::cout << "sending:" << m << std::endl;
            // send it
            p->send(m);
        }
        Timer::diff_t musec = t.stop();

        info() << double(musec) / 1e6 << "seconds";
        info() <<  repeat *1e6 / double(musec) << "messages / second";
        info() << double(size) * repeat * 1e6 / musec << "bytes / second";

    }catch(std::exception& e){
        std::cerr << "lib_test badness:" << std::endl;
        std::cerr << e.what() << std::endl;
    }catch(...){
        std::cerr << "lib_test unknown badness" << std::endl;
    }
}

