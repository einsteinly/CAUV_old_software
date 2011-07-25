
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/compoundnodes.h>
#include <gui/core/model/nodes/imagenode.h>
#include <gui/core/model/nodes/groupingnode.h>

#include <debug/cauv_debug.h>

#include <QWidget>
#include <QDragEnterEvent>
#include <QDropEvent>

#include "datastreamdragging.h"


using namespace cauv;
using namespace cauv::gui;

void NodeDropListener::dragEnterEvent(QDragEnterEvent *event)
{
    event->acceptProposedAction();
}

void NodeDropListener::dropEvent(QDropEvent *event)
{
    event->acceptProposedAction();

    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source) {
        info() << "Drag receieved from " << source;

        std::vector<boost::shared_ptr<NodeBase> >  streams = source->getDroppedNodes();
        BOOST_FOREACH(boost::shared_ptr<NodeBase> node, streams) {
            routeStream(node);
        }
    }
}

bool NodeDropListener::routeStream(boost::shared_ptr<NodeBase> s){

    info() << "Routing stream" << s->nodeName();

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
