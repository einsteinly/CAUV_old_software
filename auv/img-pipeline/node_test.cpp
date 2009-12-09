#include <iostream>
#include <string>

#include "file_input_node.h"

int main(){
	Scheduler s;
	FileInputNode n(s);
    
    n.setParam<std::string>("filename", "test.jpg");
    std::cout << "set filename=" << n.param<std::string>("filename") << std::endl;
    
    n->exec();
    
    Node::image_ptr_t img = n.getOutputImage("image");
    
}

