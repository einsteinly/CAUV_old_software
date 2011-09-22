#include "aiNode.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QRectF>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItemGroup>
#include <QDebug>

#include <debug/cauv_debug.h>

#include "elements/style.h"
#include "elements/button.h"

using namespace cauv;
using namespace cauv::gui;


struct Cross : public QGraphicsLineItem {
    Cross(qreal size) : QGraphicsLineItem(0, 0, size, size) {
        m_line = new QGraphicsLineItem(0, size, size, 0, this);
    }

    void setPen(const QPen &pen){
        m_line->setPen(pen);
        QGraphicsLineItem::setPen(pen);
    }

    void setSize(qreal size){
        m_line->setLine(0, size, size, 0);
        setLine(0, 0, size, size);
    }

private:
    void setLine(qreal x1, qreal y1, qreal x2, qreal y2){
        QGraphicsLineItem::setLine(x1, y1, x2, y2);
    }
protected:
    QGraphicsLineItem * m_line;
};


void AINode::setSize(QSizeF const& s){
    base_t::setSize(s);
    Q_EMIT boundriesChanged();
}


AINode::AINode(QGraphicsItem *parent) :
        GraphicsWindow(AI_Node_Style, parent),
        ConnectableInterface()
{
    setSize(QSizeF(200,200));
    //setCursor(Qt::ArrowCursor);
    //setFlag(ItemIsSelectable);
    connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
    
    /*
    // close button
    Cross * cross = new Cross(8);
    cross->setPos(6, 6);
    QPen pen (m_backgroundPen.color().darker(105));
    pen.setCapStyle(Qt::RoundCap);
    pen.setWidthF(2.2);
    cross->setPen(pen);
    m_closeButton = new Button(QRectF(0,0,20,20), cross);
    addButton(m_closeButton);
    connect(m_closeButton, SIGNAL(pressed()), this, SLOT(close()));
    */
    connect(this, SIGNAL(closed(GraphicsWidget*)), this, SIGNAL(disconnected()));
}

AINode::~AINode(){
    debug(2) << "~AINode()";
}


QGraphicsObject * AINode::asQGraphicsObject(){
    return this;
}
/*
void AINode::mousePressEvent(QGraphicsSceneMouseEvent *event){
    QGraphicsItem::mousePressEvent(event);
}

void AINode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    QGraphicsItem::mouseReleaseEvent(event);
}*/

