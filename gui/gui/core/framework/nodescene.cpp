#include "nodescene.h"

#include <debug/cauv_debug.h>

#include <QTextStream>
#include <QGraphicsView>
#include <QStyleOptionGraphicsItem>
#include <QDebug>

#include "../model/node.h"

using namespace cauv;
using namespace cauv::gui;

#define GUI_DEBUG_POSITION true



VanishingTextItem::VanishingTextItem(QString &text, float lod) : QGraphicsTextItem(text), m_lod(lod){
}

void VanishingTextItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){

    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if(lod > m_lod)
        QGraphicsTextItem::paint(painter, option, widget);
}


NodeScene::NodeScene(QObject * parent) : QGraphicsScene(parent)
{
    int sceneSize = 30000;

    registerDropHandler(boost::make_shared<ExampleDropHandler>());

    // a special background element that recieves drops and other events that aren't
    // accepted by items futher up the tree
    NodeSceneDropArea * dropArea = new NodeSceneDropArea(this);
    connect(this, SIGNAL(sceneRectChanged(QRectF)), dropArea, SLOT(updateGeometry(QRectF)));
    dropArea->setAcceptedMouseButtons(0);
    addItem(dropArea);

    // background lines
    for(int x = -sceneSize; x < sceneSize; x = x + 20) {
        int colour = 241;
        if(x % 100 == 0) colour = 238;
        addLine(-sceneSize, x, sceneSize, x, QPen(QColor(colour, colour, colour)));
    }
    for(int y = -sceneSize; y < sceneSize; y = y + 20) {
        int colour = 241;
        if(y % 100 == 0) colour = 238;
        addLine(y, -sceneSize, y, sceneSize, QPen(QColor(colour, colour, colour)));
    }

    // write on positions for debugging.
    // don't use in production as it's really slow
    if(GUI_DEBUG_POSITION) {
        for(int x = -sceneSize; x < sceneSize; x = x + 250) {
            for(int y = -sceneSize; y < sceneSize; y = y + 250) {
                //addEllipse(x-1, y-1, 2, 2, QPen(Qt::blue), QBrush(Qt::blue));

                QString pointString;
                QTextStream stream(&pointString);
                stream << "(" << x << "," << y << ")";
                QGraphicsTextItem* item = new VanishingTextItem(pointString, 0.2);
                item->setDefaultTextColor(QColor(210, 210, 210));
                item->setPos(x, y);
                addItem(item);
            }
        }
    }
}

NodeScene::~NodeScene(){
    debug(2) << "~NodeScene()";
}

void NodeScene::registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > handler){
    m_handlers.push_back(handler);
}

bool NodeScene::accepts(boost::shared_ptr<NodeBase>node){
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler, m_handlers) {
        if(handler->accepts(node)) return true;
    }
    return false;
}

void NodeScene::onNodeDroppedAt(boost::shared_ptr<NodeBase> node, QPointF pos){
    try {
        QGraphicsItem *  item = applyHandlers(node);
        if(item->scene() == this){
            foreach(QGraphicsView * view , this->views()){
                view->centerOn(item);
            }
        } else {
            item->setPos(pos - (item->boundingRect().center().toPoint()));
            addItem(item);
        }
    } catch (drop_not_handled){
        error() << node->nodeName() << "not supported in this drop area (" << this << ")";
    }
}

QGraphicsItem *  NodeScene::applyHandlers(boost::shared_ptr<NodeBase> node)
{
    BOOST_FOREACH(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler, m_handlers) {
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
