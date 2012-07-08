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

#include <boost/thread.hpp>

#include <debug/cauv_debug.h> 
#include <utility/bash_cout.h>

static BashColour colours[] = {
    BashColour::Red,
    BashColour::Green,
    BashColour::Cyan,
    BashColour::Blue,
    BashColour::Purple
};

struct printDebugStuff{
    void operator()(){
        int i = 0;
        while(true){
            debug() << colours[i % (sizeof(colours)/sizeof(BashColour))]
                    << i << "," << i+1 << "=" << "(" << i
                    << colours[(i+3) % (sizeof(colours)/sizeof(BashColour))]
                    << "," << i+1 << ")";
            i++;
            //boost::this_thread::sleep(boost::posix_time::milliseconds(5)); 
        }
    }
};

struct printInfoStuff{
    void operator()(){
        int i = 0;
        while(true){
            info() << colours[i % (sizeof(colours)/sizeof(BashColour))]
                    << i << "," << i+1 << "=" << "(" << i
                    << colours[(i+3) % (sizeof(colours)/sizeof(BashColour))]
                    << "," << i+1 << ")";
            i++;
            //boost::this_thread::sleep(boost::posix_time::milliseconds(5)); 
        }
    }
};

struct printErrorStuff{
    void operator()(){
        int i = 0;
        while(true){
            error() << colours[i % (sizeof(colours)/sizeof(BashColour))]
                    << i << "," << i+1 << "=" << "(" << i
                    << colours[(i+3) % (sizeof(colours)/sizeof(BashColour))]
                    << "," << i+1 << ")";
            i++;
            //boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        }
    }
};

int main(){
    boost::thread t1 = boost::thread(printDebugStuff());
    boost::thread t2 = boost::thread(printErrorStuff());
    boost::thread t3 = boost::thread(printInfoStuff());
    t1.join();
    t2.join();
    t3.join();
}

