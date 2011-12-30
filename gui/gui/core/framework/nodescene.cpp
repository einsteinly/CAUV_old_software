/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "nodescene.h"

#include <debug/cauv_debug.h>

#include <QGraphicsView>
#include <QStyleOptionGraphicsItem>
#include <QDebug>

#include "model/node.h"
#include "model/nodes/groupingnode.h"
#include "model/model.h"
#include "nodepicker.h"

#include <liquid/node.h>

#include <gui/core/framework/elements/style.h>

using namespace cauv;
using namespace cauv::gui;

#define GUI_DEBUG_POSITION false


VanishingTextItem::VanishingTextItem(QString const& text, float lod) : QGraphicsTextItem(text), m_lod(lod){
}

void VanishingTextItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){

    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if(lod > m_lod)
        QGraphicsTextItem::paint(painter, option, widget);
}


VanishingLineItem::VanishingLineItem( float lod, QGraphicsItem * parent) :
        QGraphicsLineItem(parent), m_lod(lod) {
}

VanishingLineItem::VanishingLineItem ( float lod, const QLineF & line, QGraphicsItem * parent) :
        QGraphicsLineItem(line, parent), m_lod(lod) {
}

VanishingLineItem::VanishingLineItem ( float lod, qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem * parent) :
        QGraphicsLineItem(x1,y1,x2,y2, parent), m_lod(lod) {
}

void VanishingLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){

    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if(lod > m_lod)
        QGraphicsLineItem::paint(painter, option, widget);
}


NodeScene::NodeScene(QObject * parent) : QGraphicsScene(parent)
{
    int sceneSize = 30000;

    // a special background element that recieves drops and other events that aren't
    // accepted by items futher up the tree
    NodeSceneDropArea * dropArea = new NodeSceneDropArea(this);
    connect(this, SIGNAL(sceneRectChanged(QRectF)), dropArea, SLOT(updateGeometry(QRectF)));
    dropArea->setAcceptedMouseButtons(0);
    addItem(dropArea);

    // background lines
    for(int x = -sceneSize; x < sceneSize; x = x + 25) {
        int colour = 242;
        //if(x % 100 == 0) colour = 238;
        VanishingLineItem * line = new VanishingLineItem(0.01, -sceneSize, x, sceneSize, x);
        line->setPen(QPen(QColor(colour, colour, colour)));
        this->addItem(line);
    }
    for(int y = -sceneSize; y < sceneSize; y = y + 25) {
        int colour = 242;
        //if(y % 100 == 0) colour = 238;
        VanishingLineItem * line = new VanishingLineItem(0.01, y, -sceneSize, y, sceneSize);
        line->setPen(QPen(QColor(colour, colour, colour)));
        this->addItem(line);
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

void NodeScene::registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler){
    m_handlers.push_back(handler);
}

void NodeScene::removeDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler){
    if(std::find(m_handlers.begin(), m_handlers.end(), handler)!=m_handlers.end())
        m_handlers.erase(std::find(m_handlers.begin(), m_handlers.end(), handler));
}

bool NodeScene::accepts(boost::shared_ptr<Node> const& node){
    foreach(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler, m_handlers) {
        if(handler->accepts(node)) return true;
    }
    return false;
}

void NodeScene::onNodeDroppedAt(boost::shared_ptr<Node> const& node, QPointF pos){
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

QGraphicsItem *  NodeScene::applyHandlers(boost::shared_ptr<Node> const& node)
{
    foreach(boost::shared_ptr<DropHandlerInterface<QGraphicsItem * > > const& handler, m_handlers) {
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
