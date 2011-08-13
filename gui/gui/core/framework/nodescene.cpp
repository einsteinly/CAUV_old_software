#include "nodescene.h"

#include <debug/cauv_debug.h>

#include "../model/node.h"

using namespace cauv;
using namespace cauv::gui;

NodeScene::NodeScene(QObject * parent) : QGraphicsScene(parent)
{
    registerDropHandler(boost::make_shared<ExampleDropHandler>());
}

void NodeScene::registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > handler){
    m_handlers.push_back(handler);
}

void NodeScene::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
    debug(2) << "dragEnterEvent";
    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source) {
        debug(2) << "drag came from a NodeDragSource";
        if(hasHandlerFor(source->getDroppedNodes())){
            debug(2) << "A handler is registered for this node";
            event->acceptProposedAction();
            event->accept();
        }
    }
}

void NodeScene::dropEvent(QGraphicsSceneDragDropEvent *event) {
    QGraphicsScene::dropEvent(event);
    debug() << "dropEvent";
    if(!event->isAccepted()){
        NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
        if(source) {
            if(hasHandlerFor(source->getDroppedNodes())) {
                event->acceptProposedAction();
                onDrop(source);
            } else event->ignore();
        }
    }
}

void NodeScene::dragMoveEvent(QGraphicsSceneDragDropEvent *event){
    QGraphicsScene::dragMoveEvent(event);
    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source && hasHandlerFor(source->getDroppedNodes())) {
        event->acceptProposedAction();
    }
}

void NodeScene::onNodeDropped(boost::shared_ptr<NodeBase> node){
    try {
        addItem(applyHandlers(node));
    } catch (drop_not_handled){
        error() << node->nodeName() << "not supported in this drop area (" << this << ")";
    }
}

QGraphicsItem * NodeScene::applyHandlers(boost::shared_ptr<NodeBase> node)
{
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem*> > const& handler, m_handlers) {
        try {
            // accept the first handler that matches
            if(handler->willHandle(node))
                return handler->handle(node);
        } catch (drop_not_handled){
            debug(5) << "Handler not appropriate";
        }
    }
    // no registered handler matched. oh dear.
    throw drop_not_handled();
}

bool NodeScene::hasHandlerFor(boost::shared_ptr<NodeBase> node)
{
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem*> > const& handler, m_handlers) {
        if(handler->willHandle(node)) return true;
    }
    return false;
}

bool NodeScene::hasHandlerFor(std::vector<boost::shared_ptr<NodeBase> > nodes)
{
    BOOST_FOREACH(boost::shared_ptr<NodeBase> const& node, nodes) {
        if(hasHandlerFor(node)) return true;
    }
    return false;
}
