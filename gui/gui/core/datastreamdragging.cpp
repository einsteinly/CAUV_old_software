
#include "model/nodes/numericnode.h"
#include "model/nodes/compoundnodes.h"
#include "model/nodes/imagenode.h"
#include "model/nodes/groupingnode.h"

#include <debug/cauv_debug.h>

#include "datastreamdragging.h"

using namespace cauv;
using namespace cauv::gui;

/*
void NodeDropListener::dragEnterEvent(QDragEnterEvent *event)
{
    event->acceptProposedAction();
}

void NodeDropListener::dropEvent(QDropEvent *event)
{
    event->acceptProposedAction();

    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source) {
        onDrop(source);
    }
}*/

void NodeDropListener::onDrop(NodeDragSource * source)
{
    info() << "Drag receieved from " << source;

    std::vector<boost::shared_ptr<NodeBase> >  streams = source->getDroppedNodes();
    BOOST_FOREACH(boost::shared_ptr<NodeBase> node, streams) {
        routeNode(node);
    }
}

bool NodeDropListener::routeNode(boost::shared_ptr<NodeBase> s){

    info() << "Routing stream" << s->nodeName();

    onNodeDropped(s);

    switch (s->type){
    case GuiNodeType::NumericNode:
        onNodeDropped(boost::static_pointer_cast<NumericNode>(s));
        break;
    case GuiNodeType::ImageNode:
        onNodeDropped(boost::static_pointer_cast<ImageNode>(s));
        break;
    case GuiNodeType::FloatYPRNode:
        onNodeDropped(boost::static_pointer_cast<FloatYPRNode>(s));
        break;
    case GuiNodeType::FloatXYZNode:
        onNodeDropped(boost::static_pointer_cast<FloatXYZNode>(s));
        break;
    case GuiNodeType::GroupingNode:
        onNodeDropped(boost::static_pointer_cast<GroupingNode>(s));
        break;
    default:
        error() << "Unknown node type dropped";
        return false;
    }

    return true;
}
