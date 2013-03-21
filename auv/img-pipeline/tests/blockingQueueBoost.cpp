#include <iostream>

#include <boost/thread.hpp>

#include "blockingQueueBoost.h"

BlockingQueue<int> bq;

void pusher(){
    for(int i = 0; i < 40; i++){
        bq.push(42 + i);
        bq.push(0xbeef + i);
        bq.push(1e9 + i);
        
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
}

int main()
{
    std::cout << "--- blockingQueueBoost test ---" << std::endl;

    bq.push(3);
    bq.push(4);
    bq.push(5);
    bq.push(6);
    
    int x;
    for(int i = 0; i < 5; i++){
        x = -1;
        std::cout << bq.tryPop(x, false) << ", " << std::flush;
        std::cout << x << std::endl;
    }
    
    boost::thread t = boost::thread(pusher);
    bq.push(1);
    bq.push(2);
    
    for(int i = 0; i < 20; i++){
        x = -1;
        std::cout << "try: " << bq.tryPop(x, false) << ", " << std::flush;
        std::cout << x << std::flush;
        std::cout << ", wait: " << bq.popWait() << std::endl;
    }
    std::cout << "Waiting on pusher thread..." << std::flush;
    t.join();
    std::cout << " joined." << std::endl;
    
    std::cout << "cleanup, wait: " << bq.popWait() << std::endl;
    x = -1;
    std::cout << "cleanup, wait: " << bq.tryPop(x, true) << std::flush;
    std::cout << x << std::endl;
    
    for(bool success = true; success;){
        x = -1;
        success = false;
        std::cout << "cleanup, try: " << (success = bq.tryPop(x, false)) << ", " << std::flush;
        std::cout << x << std::endl;
    }
	
	return 0;
}
