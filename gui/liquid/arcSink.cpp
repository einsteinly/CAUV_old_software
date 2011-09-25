#include "arcSink.h"

#include <QPropertyAnimation>
#include <QGraphicsBlurEffect>

using namespace liquid;

ArcSink::ArcSink(ArcStyle const& of_style,
    CutoutStyle const& with_cutout,
    ConnectionSink *connectionDelegate)
    : QGraphicsObject(),
      QGraphicsLayoutItem(),
      ConnectionSink(),
      RequiresCutout(),
      m_arc_style(of_style),
      m_cutout_style(with_cutout),
      m_connectionDelegate(connectionDelegate),
      m_highlight(new QGraphicsEllipseItem()),
      m_rect(
        0,
        -m_cutout_style.main_cutout.cutout_base/2,
        m_cutout_style.main_cutout.cutout_depth,
        m_cutout_style.main_cutout.cutout_base
      ){
    
    m_highlight->setRect(m_rect);
    m_highlight->setPen(Qt::NoPen);
    m_highlight->setBrush(QBrush(QColor(50,255,50,128)));

    setGraphicsEffect(new QGraphicsBlurEffect());

    setSizePolicy(QSizePolicy::Fixed);

    setFlag(ItemHasNoContents);
    
    connect(this, SIGNAL(xChanged()), this, SIGNAL(geometryChanged()));
    connect(this, SIGNAL(yChanged()), this, SIGNAL(geometryChanged()));
}

bool ArcSink::willAcceptConnection(void* from_source){
    return m_connectionDelegate->willAcceptConnection(from_source);
}

void ArcSink::doPresentHighlight(qreal intensity){
    QPropertyAnimation *fadeHL = new QPropertyAnimation(this, "opacity");
    fadeHL->setEndValue(intensity);
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

