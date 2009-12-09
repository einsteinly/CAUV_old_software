#include <iostream>
#include <string>
#include <exception>

#include "nodes/fileInputNode.h"
#include "nodes/fileOutputNode.h"
#include "nodes/localDisplayNode.h"
#include "nodes/copyNode.h"
#include "nodes/resizeNode.h"

int main(){
	Scheduler s;
    
    // Nodes
    node_ptr_t in_node(new FileInputNode(s));
    node_ptr_t copy_node(new CopyNode(s));
    node_ptr_t out_node(new FileOutputNode(s));
    node_ptr_t display_node(new LocalDisplayNode(s));
    node_ptr_t out_node_small(new FileOutputNode(s));
    node_ptr_t resize_node(new ResizeNode(s));
    
    // Arcs
    in_node->setOutput("image_out", out_node, "image_in");
    out_node->setInput("image_in", in_node, "image_out");

    in_node->setOutput("image_out", copy_node, "image_in");
    copy_node->setInput("image_in", in_node, "image_out");

    copy_node->setOutput("image_out_A", display_node, "image_in");
    display_node->setInput("image_in", copy_node, "image_out_A");

    copy_node->setOutput("image_out_B", resize_node, "image_in");
    resize_node->setInput("image_in", copy_node, "image_out_B");
    
    resize_node->setOutput("image_out", out_node_small, "image_in");
    out_node_small->setInput("image_in", resize_node, "image_out");
    
    // Parameters
    in_node->setParam<std::string>("filename", "test.jpg");
    std::cout << "set filename=" << in_node->param<std::string>("filename") << std::endl;
    
    out_node->setParam<std::string>("filename", "out.jpg");
    out_node->setParam<int>("jpeg quality", 90);

    resize_node->setParam<float>("scale factor", 0.2);

    out_node_small->setParam<std::string>("filename", "out-small.jpg");
    out_node_small->setParam<int>("jpeg quality", 100);
    
    // Go!
    std::cerr << "exec fin node" << std::endl;
    in_node->exec();
    //boost::shared_ptr<Image> img = in_node->getOutputImage("image_out");

    std::cerr << "exec fout node" << std::endl;
    out_node->exec();
    
    // Expect this to fail - input hasn't 
    std::cerr << "exec display node (Expecting error)" << std::endl;
    try{
        display_node->exec();
    } catch(std::exception& e){
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    std::cerr << "exec copy node" << std::endl;
    copy_node->exec();

    std::cerr << "exec display node" << std::endl;
    display_node->exec();

    std::cerr << "exec resize node" << std::endl;
    resize_node->exec();

    std::cerr << "exec fout small node" << std::endl;
    out_node_small->exec();

    std::cout << "Press enter to exit." << std::endl;
    std::cin.getline(NULL, 0);
    std::cerr << "done." << std::endl;
    
    return EXIT_SUCCESS;
}

