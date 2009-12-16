#include <iostream>
#include "blockingQueueBoost.h"

int main()
{
    std::cout << "--- blockingQueueBoost test ---" << std::endl;

	BlockingQueue <int> bq;
	
    bq.push(3);
    bq.push(4);
    bq.push(5);
    bq.push(6);

    for(int i = 0; i < 5; i++){
        int x = -1;
        std::cout << bq.trypop(x, false) << ", ";
        std::cout << x << std::endl;
    }

    bq.push(1);
    bq.push(2);
    
    for(int i = 0; i < 5; i++){
        int x = -1;
        std::cout << bq.trypop(x, false) << ", ";
        std::cout << x << std::endl;
    }
	
	return 0;
}
