#include "imageProcessor.h"

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
    // TODO
}

void ImageProcessor::onRemoveNodeMessage(RemoveNodeMessage const& m){
    // TODO
}

void ImageProcessor::onSetNodeParameterMessage(SetNodeParameterMessage const& m){
    // TODO
}

/**
 * Main event loop.
 *
void main() throw()
{


    switch message
    {
        case addNode:
            // notePtr node(  call node factory
            this->m_last_id++; 
            this->m_node_map[this->m_last_id] = node;
            
            // send message containing this id
            break;
        
       case removeNode:
           // node_id = get from message
           try
           {
               this->m_node_map[node_id]->remove(); // Calls the node to remove itself by removing any inputs
               this->m_node_map.erase(node_id); // Erases the node from the map
           }
           catch (...)
           {
               //send a fail message    
           }
           break;
     }
}*/


ImageProcessor::~ImageProcessor(){
    m_scheduler.stopWait();
}


