#include <iostream>
#include <string>

#include "nodes/fileInputNode.h"

int main(){
	Scheduler s;
	FileInputNode n(s);
    
    n.setParam<std::string>("filename", "test.jpg");
    std::cout << "set filename=" << n.param<std::string>("filename") << std::endl;
    
    n.exec();
    
    boost::shared_ptr<Image> img = n.getOutputImage("image");
    
}

