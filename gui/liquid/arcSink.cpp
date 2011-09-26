#include "arcSink.h"

#include <QPropertyAnimation>
#include <QGraphicsBlurEffect>

#include <debug/cauv_debug.h>

using namespace liquid;

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
    
    QGraphicsBlurEffect *blur = new QGraphicsBlurEffect();
    blur->setBlurRadius(3.0);
    setGraphicsEffect(blur);

    /*QGraphicsPathItem *test_item = new QGraphicsPathItem(this);
    QPainterPath p;
    p.addRect(m_rect);
    test_item->setPath(p);
    test_item->setBrush(Qt::NoBrush);
    test_item->setPen(QPen(QColor(20,200,20,128)));
    */

    setSizePolicy(QSizePolicy::Fixed);

    setFlag(ItemHasNoContents);
    
    // start out not presenting a highlight:
    doPresentHighlight(0);
    
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
}

bool ArcSink::willAcceptConnection(void* from_source){
    return m_connectionDelegate->willAcceptConnection(from_source);
}

void ArcSink::doPresentHighlight(qreal intensity){
    debug(8) << "doPresentHighlight:" << intensity;
    QPropertyAnimation *fadeHL = new QPropertyAnimation(this, "opacity");
    // !!! can't set zero opacity, because that renders this item invisible,
    // and then it won't be picked up by the proximity test!
    fadeHL->setEndValue(std::max(intensity, 0.01));
    fadeHL->setDuration(100);    
    fadeHL->start();
}

ArcSink::ConnectionStatus ArcSink::doAcceptConnection(void* from_source){
    return m_connectionDelegate->doAcceptConnection(from_source);
}

// RequiresCutout:
QList<CutoutStyle> ArcSink::cutoutGeometry() const{
    QList<CutoutStyle> r;
    return r << m_cutout_style;
}

// QGraphicsLayoutItem:
void ArcSink::setGeometry(QRectF const& rect){
    setPos(rect.topLeft() - m_rect.topLeft());
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
    Q_UNUSED(painter);
    Q_UNUSED(option);
    Q_UNUSED(widget);
}

