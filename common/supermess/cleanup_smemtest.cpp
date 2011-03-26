#include <boost/interprocess/shared_memory_object.hpp>

#include "shared_mem_test_lib.h"

int main(){
    boost::interprocess::shared_memory_object::remove(Alloc_Name);
}

