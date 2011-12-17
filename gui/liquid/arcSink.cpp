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

#include "arcSink.h"

#include <QPropertyAnimation>
#include <QGraphicsBlurEffect>
#include <QPainter>

#include <debug/cauv_debug.h>

using namespace liquid;

AbstractArcSink::AbstractArcSink(QGraphicsItem * parent): QGraphicsObject(parent), ConnectionSink() {
    debug(7) << "AbstractArcSink()";
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
    connectParentSignals(parent);
}
AbstractArcSink::~AbstractArcSink(){
    debug(7) << "~AbstractArcSink(): scene=" << scene(); 
    Q_EMIT(disconnected(this));
}

void AbstractArcSink::setParentItem(QGraphicsItem* item){
    disconnectParentSignals(parentItem());
    connectParentSignals(item);
    QGraphicsObject::setParentItem(item);
}

void AbstractArcSink::disconnectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    if(parent){
        disconnect(parent, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(parentChanged()), this, SIGNAL(geometryChanged()));
    }
}

void AbstractArcSink::connectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    debug(7) << "connectParentSignals:" << parent;
    if(parent){
        connect(parent, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
        connect(parent, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
        connect(parent, SIGNAL(parentChanged()), this, SIGNAL(geometryChanged()));
    }
}

ArcSink::ArcSink(ArcStyle const& of_style,
    CutoutStyle const& with_cutout,
    ConnectionSink *connectionDelegate)
    : AbstractArcSink(),
      QGraphicsLayoutItem(),
      RequiresCutout(),
      m_arc_style(of_style),
      m_cutout_style(with_cutout),
      m_connectionDelegate(connectionDelegate),
      m_highlight(new QGraphicsEllipseItem(this)),
      m_rect(
        0,
        -(1+m_cutout_style.main_cutout.cutout_base/2),
        m_cutout_style.main_cutout.cutout_depth,
        2+m_cutout_style.main_cutout.cutout_base
      ){
    
    m_highlight->setRect(m_rect.adjusted(2,2,-2,-2));
    m_highlight->setPen(Qt::NoPen);
    m_highlight->setBrush(QBrush(QColor(50,255,50,160)));
    
    // FIXME: !!! adding this graphics effect causes a segfault (somtimes) when
    // nodes containing arcsinks are removed from the scene: this is probably
    // something to do with the blur changing the bounding rect (segfault
    // occurs in BSP tree walking)
    /*QGraphicsBlurEffect *blur = new QGraphicsBlurEffect();
    blur->setBlurRadius(3.0);
    setGraphicsEffect(blur);*/

    /*QGraphicsPathItem *test_item = new QGraphicsPathItem(this);
    QPainterPath p;
    p.addRect(m_rect);
    test_item->setPath(p);
    test_item->setBrush(Qt::NoBrush);
    test_item->setPen(QPen(QColor(20,200,20,128)));
    */

    QGraphicsLayoutItem::setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    // start out not presenting a highlight:
    doPresentHighlight(0);
}
ArcSink::~ArcSink(){
    debug(7) << "~ArcSink()"; 
}

bool ArcSink::willAcceptConnection(ArcSourceDelegate* from_source){
    return m_connectionDelegate->willAcceptConnection(from_source);
}

void ArcSink::doPresentHighlight(qreal intensity){
    // !!! TODO: could probably do away with animation here - highlights will
    // change smoothly anyway
    debug(8) << "doPresentHighlight:" << intensity;
    QPropertyAnimation *fadeHL = new QPropertyAnimation(this, "opacity");
    // !!! can't set zero opacity, because that renders this item invisible,
    // and then it won't be picked up by the proximity test!
    fadeHL->setEndValue(std::max(intensity, 0.01));
    fadeHL->setDuration(100);    
    fadeHL->start();
}

ArcSink::ConnectionStatus ArcSink::doAcceptConnection(ArcSourceDelegate* from_source){
    return m_connectionDelegate->doAcceptConnection(from_source);
}

// RequiresCutout:
QList<CutoutStyle> ArcSink::cutoutGeometry() const{
    QList<CutoutStyle> r;
    return r << m_cutout_style;
}

// QGraphicsLayoutItem:
void ArcSink::setGeometry(QRectF const& rect){
    debug(9) << "ArcSink::setGeometry" << rect.topLeft().x() << rect.topLeft().y() << rect.width() << rect.height();
    setPos(rect.topLeft() - m_rect.topLeft());
    //QGraphicsLayoutItem::setGeometry(rect);
}

QSizeF ArcSink::sizeHint(Qt::SizeHint which,
                         const QSizeF&
                         constraint) const{
    return m_rect.size();
}

QRectF ArcSink::boundingRect() const{
    return m_rect.adjusted(-1, -1, 1, 1);
}

void ArcSink::paint(QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(QPen(QColor(180,10,120,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
}

