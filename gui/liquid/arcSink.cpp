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

AbstractArcSink::AbstractArcSink(QGraphicsItem * parent)
    : QGraphicsObject(parent),
      ConnectionSink(),
      LayoutItems(this){
    debug(7) << "AbstractArcSink()";
    // !!! TODO: instead of signals we can use ItemScenePositionHasChanged notifications    
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
    connectParentSignals(parent);
}
AbstractArcSink::~AbstractArcSink(){
    debug(7) << "~AbstractArcSink(): scene=" << scene(); 
    Q_EMIT(disconnected(this));
    LayoutItems::unRegisterSinkItem(this);
}

void AbstractArcSink::setParentItem(QGraphicsItem* item){
    disconnectParentSignals(parentItem());
    connectParentSignals(item);
    QGraphicsObject::setParentItem(item);
}

QGraphicsItem* AbstractArcSink::ultimateParent(){
    QGraphicsItem* last_parent = this;
    QGraphicsItem* parent = NULL;
    while((parent = last_parent->parentItem()))
        last_parent = parent;
    return last_parent;
}

void AbstractArcSink::disconnectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    if(parent){
        // !!! TODO: instead of signals we can use ItemScenePositionHasChanged notifications
        disconnect(parent, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
        disconnect(parent, SIGNAL(parentChanged()), this, SIGNAL(geometryChanged()));
    }
}

void AbstractArcSink::connectParentSignals(QGraphicsItem* p){
    QGraphicsObject* parent = dynamic_cast<QGraphicsObject*>(p);
    debug(7) << "connectParentSignals:" << parent;
    if(parent){
        // !!! TODO: instead of signals we can use ItemScenePositionHasChanged notifications
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
      m_highlight(new QGraphicsPathItem(this)),
      m_back(new QGraphicsPathItem(this)),
      m_rect(
        0,
        -(1+m_cutout_style.main_cutout.cutout_base/2),
        m_cutout_style.main_cutout.cutout_depth,
        2+m_cutout_style.main_cutout.cutout_base
      ){
    
    
    QPainterPath hl_path(m_rect.topLeft() - QPointF(9, 3));
    hl_path.lineTo(m_rect.right(), 1);
    hl_path.lineTo(m_rect.bottomLeft() - QPointF(9, -3));
    hl_path.lineTo(m_rect.topLeft() - QPointF(9, 3));

    //m_highlight->setRect(m_rect.adjusted(2,2,-2,-2));
    m_highlight->setPen(Qt::NoPen);
    m_highlight->setBrush(QBrush(QColor(50,255,50,160)));
    m_highlight->setPath(hl_path);

    m_highlight->setZValue(10);
    
    /* |
     * |<--------->| cutout_depth
     *  "-_                         -
     *      "-_                     |
     *          "-_  -              |
     *             | | cutout_tip   | cutout_base
     *          _-"  -              |
     *      _-"                     |
     *  _-"                         -
     * |
     * |
     */
    float main_depth = m_cutout_style.main_cutout.cutout_depth;
    float main_tip = m_cutout_style.main_cutout.cutout_tip;
    float main_base = m_cutout_style.main_cutout.cutout_base;
    float second_depth = m_cutout_style.second_cutout.cutout_depth;
    float second_tip = m_cutout_style.second_cutout.cutout_tip;
    float second_base = m_cutout_style.second_cutout.cutout_base;
    
    // pixelcentrestupidalignmenthackhacksorryabouthis
    QPointF offset(-0.5, 0.5);

    QPainterPath back_path;
    back_path.moveTo(offset + QPointF(0,          -main_base/2));

    back_path.lineTo(offset + QPointF(main_depth, -main_tip/2));
    back_path.lineTo(offset + QPointF(main_depth,  main_tip/2));
    back_path.lineTo(offset + QPointF(0,           main_base/2));

    back_path.lineTo(offset + QPointF(0,             second_base/2));
    back_path.lineTo(offset + QPointF(second_depth,  second_tip/2));
    back_path.lineTo(offset + QPointF(second_depth, -second_tip/2));
    back_path.lineTo(offset + QPointF(0,            -second_base/2));

    back_path.lineTo(offset + QPointF(0,          -main_base/2));    

    m_back->setPen(m_cutout_style.style.pen);
    m_back->setBrush(m_cutout_style.style.brush);
    m_back->setPath(back_path);

    m_back->setZValue(-10);
    m_back->setFlag(ItemStacksBehindParent);

    
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

    #if !defined(CAUV_DEBUG_DRAW_LAYOUT)
    setFlag(ItemHasNoContents);
    #endif // !defined(CAUV_DEBUG_DRAW_LAYOUT)
}
ArcSink::~ArcSink(){
    debug(7) << "~ArcSink()"; 
}

bool ArcSink::willAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink * to_sink){
    return m_connectionDelegate->willAcceptConnection(from_source, to_sink);
}

void ArcSink::doPresentHighlight(qreal intensity){
    debug(8) << "doPresentHighlight:" << intensity;
    m_highlight->setOpacity(std::max(intensity, 0.01));
}

ArcSink::ConnectionStatus ArcSink::doAcceptConnection(ArcSourceDelegate* from_source, AbstractArcSink* to_sink){
    return m_connectionDelegate->doAcceptConnection(from_source, to_sink);
}

// RequiresCutout:
QList<CutoutStyle> ArcSink::cutoutGeometry() const{
    QList<CutoutStyle> r;
    return r << m_cutout_style;
}

// QGraphicsLayoutItem:
void ArcSink::setGeometry(QRectF const& rect){
    debug(9) << "ArcSink::setGeometry" << rect.topLeft().x() << rect.topLeft().y() << rect.width() << rect.height();
    QGraphicsLayoutItem::setGeometry(rect);    
    setPos(rect.topLeft() - m_rect.topLeft());
}

QSizeF ArcSink::sizeHint(Qt::SizeHint which,
                         const QSizeF&
                         constraint) const{
    Q_UNUSED(which);
    Q_UNUSED(constraint)
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
    Q_UNUSED(painter);
    #ifdef CAUV_DEBUG_DRAW_LAYOUT
    painter->setPen(QPen(QColor(180,10,120,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    #endif // def CAUV_DEBUG_DRAW_LAYOUT 
}

