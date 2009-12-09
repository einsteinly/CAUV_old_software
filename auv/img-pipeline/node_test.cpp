#include <iostream>
#include <string>

#include "nodes/fileInputNode.h"
#include "nodes/fileOutputNode.h"

int main(){
	Scheduler s;
    
    node_ptr_t in_node(new FileInputNode(s));
    node_ptr_t out_node(new FileOutputNode(s));
    
    in_node->setOutput(output_id("image_out"), out_node, input_id("image_in"));
    out_node->setInput("image_in", in_node, "image_out");

    in_node->setParam<std::string>("filename", "test.jpg");
    std::cout << "set filename=" << in_node->param<std::string>("filename") << std::endl;
    
    out_node->setParam<std::string>("filename", "out.jpg");
    out_node->setParam<int>("jpeg quality", 90);
    
    std::cerr << "exec fin node" << std::endl;
    in_node->exec();
    //boost::shared_ptr<Image> img = in_node->getOutputImage("image_out");

    std::cerr << "exec fout node" << std::endl;
    out_node->exec();

    std::cerr << "done." << std::endl;
    
    return EXIT_SUCCESS;
}

