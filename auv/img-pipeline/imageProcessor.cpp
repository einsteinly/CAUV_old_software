#include "imageProcessor.h"

#include "nodeFactory.h"

ImageProcessor::ImageProcessor()
    : m_scheduler(){
    m_scheduler.start();
}

void ImageProcessor::onImageMessage(ImageMessage const& m){
    std::set<input_node_ptr_t>::iterator i;
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onImageMessage(m);
}

void ImageProcessor::onAddNodeMessage(AddNodeMessage const& m){
    try{
       
        node_ptr_t node = NodeFactoryRegister::create(m.nodeType(), m_scheduler);

        BOOST_FOREACH(NodeInputArc const& a, m.parents())
            node->setInput(a.dst, _lookupNode(a.src.node), a.src.output);
        BOOST_FOREACH(NodeOutputArc const& a, m.children())
            node->setOutput(a.src, _lookupNode(a.dst.node), a.dst.input);

        m_nodes[_newID(node)] = node;

        if(node->isInputNode){
            m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
        }
    }catch(std::exception& e){
        std::cerr << "error: " << __func__ << " : " << e.what() << std::endl;
    }
    // TODO: error message of some sort, or something
    // TODO: send NodeAddedMessage with new ID?  
}

void ImageProcessor::onRemoveNodeMessage(RemoveNodeMessage const& m){
    try{
        node_ptr_t n = _lookupNode(m.nodeId());
        m_nodes.erase(m.nodeId());
       
        if(n->isInputNode){
            m_input_nodes.erase(boost::dynamic_pointer_cast<InputNode, Node>(n));
        }
        /* since the graph is linked both ways we have to unlink the node from
         * it's neighbors _and_ unlink the neigbors from the node
         */
        BOOST_FOREACH(node_ptr_t p, n->parents())
            p->clearOutputs(n);
        BOOST_FOREACH(node_ptr_t p, n->children())
            p->clearInputs(n);
        n->clearOutputs();
        n->clearInputs(); 
    }catch(std::exception& e){
        std::cerr << "error: " << __func__ << " : " << e.what() << std::endl;
    }
    // TODO: error message of some sort, or something
}

void ImageProcessor::onSetNodeParameterMessage(SetNodeParameterMessage const& m){
    try{
        // TODO: requires somewhat complicated message code for variadic values
        throw(std::runtime_error("not implemented"));//_lookupNode(m.nodeID)->setParam();
    }catch(std::exception& e){
        std::cerr << "error: " << __func__ << " : " << e.what() << std::endl;
    }
    // TODO: error message of some sort, or something
}

ImageProcessor::~ImageProcessor(){
    m_scheduler.stopWait();
}


