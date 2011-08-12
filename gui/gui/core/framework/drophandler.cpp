#include "drophandler.h"

#include "../model/nodes/numericnode.h"
#include "../model/nodes/compoundnodes.h"
#include "../model/nodes/imagenode.h"
#include "../model/nodes/groupingnode.h"

using namespace cauv;
using namespace cauv::gui;

DropHandler::DropHandler()
{
}

QGraphicsItem * DropHandler::handle(boost::shared_ptr<NumericNode> ){
    throw drop_not_handled();
}

QGraphicsItem * DropHandler::handle(boost::shared_ptr<ImageNode> ){
    throw drop_not_handled();
}

QGraphicsItem * DropHandler::handle(boost::shared_ptr<FloatYPRNode> ){
    throw drop_not_handled();
}

QGraphicsItem * DropHandler::handle(boost::shared_ptr<FloatXYZNode> ){
    throw drop_not_handled();
}

QGraphicsItem * DropHandler::handle(boost::shared_ptr<GroupingNode> ){
    throw drop_not_handled();
}
