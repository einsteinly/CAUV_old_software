#include "nodescene.h"

#include <debug/cauv_debug.h>

#include <QTextStream>
#include <QDebug>

#include "../model/node.h"

using namespace cauv;
using namespace cauv::gui;

#define GUI_DEBUG_POSITION false


NodeScene::NodeScene(QObject * parent) : QGraphicsScene(parent)
{
    int sceneSize = 30000;

    registerDropHandler(boost::make_shared<GraphDropHandler>());

    // a special background element that recieves drops and other events that aren't
    // accepted by items futher up the tree
    NodeSceneDropArea * dropArea = new NodeSceneDropArea(this);
    connect(this, SIGNAL(sceneRectChanged(QRectF)), dropArea, SLOT(updateGeometry(QRectF)));
    addItem(dropArea);

    // background lines
    for(int x = -sceneSize; x < 2*sceneSize; x = x + 20) {
        int colour = 247;
        if(x % 100 == 0) colour = 245;
        addLine(-sceneSize, x, 2*sceneSize, x, QPen(QColor(colour, colour, colour)));
    }
    for(int y = -sceneSize; y < 2*sceneSize; y = y + 20) {
        int colour = 247;
        if(y % 100 == 0) colour = 245;
        addLine(y, -sceneSize, y, 2*sceneSize, QPen(QColor(colour, colour, colour)));
    }

    // write on positions for debugging.
    // don't use in production as it's really slow
    if(GUI_DEBUG_POSITION) {
        for(int x = -sceneSize; x < 2*sceneSize; x = x + 100) {
            for(int y = -sceneSize; y < 2*sceneSize; y = y + 100) {
                //addEllipse(x-1, y-1, 2, 2, QPen(Qt::blue), QBrush(Qt::blue));

                QString pointString;
                QTextStream stream(&pointString);
                stream << "(" << x << "," << y << ")";
                QGraphicsTextItem* item = addText(pointString);
                item->setDefaultTextColor(QColor(210, 210, 210));
                item->setPos(x, y);
            }
        }
    }
}

void NodeScene::registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > handler){
    m_handlers.push_back(handler);
}

bool NodeScene::accepts(boost::shared_ptr<NodeBase>node){
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem*> > const& handler, m_handlers) {
        if(handler->accepts(node)) return true;
    }
    return false;
}

void NodeScene::onNodeDroppedAt(boost::shared_ptr<NodeBase> node, QPointF pos){
    try {
        QGraphicsItem * item = applyHandlers(node);
        item->setPos(pos - (item->boundingRect().center().toPoint()));
        addItem(item);
    } catch (drop_not_handled){
        error() << node->nodeName() << "not supported in this drop area (" << this << ")";
    }
}

QGraphicsItem * NodeScene::applyHandlers(boost::shared_ptr<NodeBase> node)
{
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem*> > const& handler, m_handlers) {
        try {
            // accept the first handler that matches
            if(handler->accepts(node))
                return handler->handle(node);
        } catch (drop_not_handled){
            debug(5) << "Handler not appropriate";
        }
    }
    // no registered handler matched. oh dear.
    throw drop_not_handled();
}

