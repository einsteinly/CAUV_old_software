#include "imgNode.h"

using cauv::gui::f::ImgNode;
using cauv::gui::f::FNode;
using cauv::gui::f::Manager;

ImgNode::ImgNode(Manager& m, node_id_t id)
    : FNode(m, id){
    
}

ImgNode::ImgNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p)
    : FNode(m, p){

}


