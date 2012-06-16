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

#include <QtGui>
#include <QPainterPath>

#include "radialSegment.h"
#include "style.h"

using namespace liquid;
using namespace liquid::magma;

class ShimmeringAnimation : public QSequentialAnimationGroup {
public:
    ShimmeringAnimation(QObject * target, const QByteArray& property,
                        QVariant amplitude, int duration):
        m_target(target), m_property(property), m_amplitude(amplitude),
        m_shimmerIn(new QPropertyAnimation(target, property, this)),
        m_shimmerOut(new QPropertyAnimation(target, property, this))
    {
        m_shimmerIn->setStartValue(target->property(property));
        m_shimmerIn->setEndValue(target->property(property).toFloat() -
                                 m_amplitude.toFloat());
        m_shimmerIn->setDuration(duration);
        m_shimmerIn->setEasingCurve(QEasingCurve::InSine);

        m_shimmerOut->setStartValue(target->property(property).toFloat() -
                                 m_amplitude.toFloat());
        m_shimmerOut->setEndValue(target->property(property));
        m_shimmerOut->setDuration(duration);
        m_shimmerOut->setEasingCurve(QEasingCurve::OutSine);

        addAnimation(m_shimmerIn);
        addAnimation(m_shimmerOut);
        setLoopCount(-1);
    }

protected:
    QObject * m_target;
    QByteArray const& m_property;
    QVariant m_amplitude;
    QPropertyAnimation * m_shimmerIn;
    QPropertyAnimation * m_shimmerOut;
};


RadialSegment::RadialSegment(RadialSegmentStyle const& style, float radius,
                             float rotation, float angle, QWidget *parent)
    : QWidget(parent), m_style(style), m_radius(radius),
      m_rotation(rotation), m_angle(angle)
{
    setFixedSize(m_radius*2,m_radius*2);

    QPropertyAnimation * radiusAnimation = new QPropertyAnimation(this, "radius", this);
    radiusAnimation->setEasingCurve(QEasingCurve::InOutQuad);
    radiusAnimation->setStartValue(m_style.width);
    radiusAnimation->setEndValue(m_radius);
    radiusAnimation->setDuration(250);
    m_startupAnimations.addAnimation(radiusAnimation);

    QPropertyAnimation * angleAnimation = new QPropertyAnimation(this, "angle", this);
    angleAnimation->setEasingCurve(QEasingCurve::InOutQuad);
    angleAnimation->setStartValue(0);
    angleAnimation->setEndValue(m_angle);
    angleAnimation->setDuration(150);
    m_startupAnimations.addAnimation(angleAnimation);

    QPropertyAnimation * rotationAnimation = new QPropertyAnimation(this, "rotation", this);
    //rotationAnimation->setEasingCurve(QEasingCurve::InOutQuad);
    rotationAnimation->setStartValue(0);
    rotationAnimation->setEndValue(m_rotation);
    rotationAnimation->setDuration(200);
    m_startupAnimations.addAnimation(rotationAnimation);

    m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "radius", 4.0, 8000));
    m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "angle", 5.0, 10000));
    m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "rotation", 20.0, 45000));
}

void RadialSegment::showEvent ( QShowEvent * event ) {
    if(!event->spontaneous())
        animateIn();
}

void RadialSegment::animateIn(){
    m_startupAnimations.start();
    connect(&m_startupAnimations, SIGNAL(finished()),
            &m_runningAnimations, SLOT(start()));
}

void RadialSegment::setRadius(float r){
    m_radius = r;
    update();
}

float RadialSegment::getRadius(){
    return m_radius;
}

void RadialSegment::setRotation(float r){
    m_rotation = r;
    update();
}

float RadialSegment::getRotation(){
    return m_rotation;
}

void RadialSegment::setAngle(float a){
    m_angle = a;
    update();
}

float RadialSegment::getAngle(){
    return m_angle;
}

void RadialSegment::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        dragPosition = event->globalPos() - frameGeometry().topLeft();
        event->accept();
    }

}

void RadialSegment::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        move(event->globalPos() - dragPosition);
        event->accept();
    }

}

void RadialSegment::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    QPainterPath path;
    painter.setRenderHint(QPainter::Antialiasing);

    //painter.fillRect(this->rect(), Qt::green);

    painter.setPen(m_style.pen);
    painter.setBrush(m_style.brush);
    painter.setFont(m_style.tick.font);

    float scale = 1.0;
    painter.scale( 1.0/scale, 1.0/scale);
    painter.translate(width() / 2, height() / 2);

    float scaledRadius = (m_radius * scale) -2; // -2 means we don't draw over
                                                // the edges of the mask which
                                                // causing aliasing artifacts

    QRectF outerElipse = QRectF(
                -scaledRadius, -scaledRadius,
                (2*scaledRadius), (2*scaledRadius));

    QRectF innerElipse = QRectF(
                -scaledRadius    + (m_style.width/2),
                -scaledRadius    + (m_style.width/2),
                (2*scaledRadius) - m_style.width,
                (2*scaledRadius) - m_style.width);

    //painter.drawEllipse(outerElipse);
   // painter.setPen(Qt::magenta);
   // painter.drawEllipse(innerElipse);

    path.arcMoveTo(innerElipse, m_rotation);
    path.arcTo(outerElipse, m_rotation, m_angle);
    path.arcTo(innerElipse, m_rotation + m_angle, - m_angle);
    path.closeSubpath();

    painter.drawPath(path);

}

QRegion RadialSegment::recomputeMask(){
    QRegion maskedRegion(0, 0, m_radius*2, m_radius*2, QRegion::Ellipse);
    setMask(maskedRegion);
    Q_EMIT maskComputed(maskedRegion);
    return maskedRegion;
}

void RadialSegment::resizeEvent(QResizeEvent * /* event */)
{
    recomputeMask();
}

QSize RadialSegment::sizeHint() const
{
    return QSize(m_radius*2, m_radius*2);
}

QModelIndex RadialSegment::indexAt(QPoint const& p){
    return QModelIndex();
}
