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

#include <math.h>
#include <algorithm>

#include "radialSegment.h"
#include "radialMenuItem.h"
#include "style.h"

#include <debug/cauv_debug.h>

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


RadialSegment::RadialSegment(RadialSegmentStyle const& style,
                             bool isRoot,
                             QWidget *parent
                             )
    : QWidget(parent),
      m_style(style),
      m_radius(0),
      m_rotation(0),
      m_angle(90),
      m_animationsSetup(false),
      m_isRoot(isRoot)
{

    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    hide();
}

void RadialSegment::animateIn(){
    if(!m_animationsSetup) {

        //todo: this needs fixing. animations need to be updated when setX methods are called

        QPropertyAnimation * radiusAnimation = new QPropertyAnimation(this, "radius", this);
        radiusAnimation->setEasingCurve(QEasingCurve::InOutQuad);
        radiusAnimation->setStartValue(m_radius - m_style.width);
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
        rotationAnimation->setEasingCurve(QEasingCurve::OutCubic);
        rotationAnimation->setStartValue(0);
        rotationAnimation->setEndValue(m_rotation);
        rotationAnimation->setDuration(450);
        m_startupAnimations.addAnimation(rotationAnimation);

        m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "radius", 4.0, 8000));
        m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "angle", 5.0, 10000));
        m_runningAnimations.addAnimation(new ShimmeringAnimation(this, "rotation", 20.0, 45000));

        m_animationsSetup = true;
    }

    show();
    m_startupAnimations.start();
    //connect(&m_startupAnimations, SIGNAL(finished()),
    //        &m_runningAnimations, SLOT(start()));
}


void RadialSegment::itemHovered(){
    QObject * object = sender();
    if(RadialMenuItem * item = qobject_cast<RadialMenuItem*>(object)){
        Q_EMIT itemHovered(item);
    }
}

void RadialSegment::itemSelected(){
    QObject * object = sender();
    if(RadialMenuItem * item = qobject_cast<RadialMenuItem*>(object)){
        Q_EMIT itemSelected(item);
    }
}

void RadialSegment::setRadius(float r){
    m_radius = r;
    resize(sizeHint()*2);
    Q_EMIT sizeChanged(size());
    relayoutItems();
    update();
}

float RadialSegment::getRadius() const{
    return m_radius;
}

void RadialSegment::setRotation(float r){
    m_rotation = r;
    relayoutItems();
    update();
}

float RadialSegment::getRotation() const{
    return m_rotation;
}

void RadialSegment::setAngle(float a){
    m_angle = a;
    relayoutItems();
    update();
}

float RadialSegment::getAngle() const{
    return m_angle;
}


void RadialSegment::relayoutItems(){
    int numItems = m_items.count();
    float totalAngle = std::min(
                std::max(numItems*m_style.preferredAnglePerItem, m_angle),
                m_style.maxAngle
                );
    if(m_isRoot)
        totalAngle = m_style.maxAngle;

    // if the size has had to change due to the above constraints the
    // paint system needs to be told, so update the angle it reads
    m_angle = totalAngle;

    float anglePerItem = m_angle / numItems;
    int count = 0;
    foreach(RadialMenuItem * item, m_items){
        // Qt's 0 degrees is east
        float itemAngle = m_rotation + (anglePerItem * count) + (anglePerItem/2) + 90;

        //debug() << itemAngle << m_rotation << m_angle;

        item->setAngle(itemAngle);
        int x = sin(itemAngle*M_PI/180) * (m_radius - (m_style.width/2));
        int y = cos(itemAngle*M_PI/180) * (m_radius - (m_style.width/2));
        item->move(x + (width()/2) - (item->width()/2),
                   y + (height()/2) - (item->height()/2));


        // set up the path for the text to follow
        QPainterPath path;
        QRectF textElipse = innerElipse().adjusted(
                    -m_style.width/2, -m_style.width/2,
                    m_style.width/2, m_style.width/2);
//        textElipse = textElipse.translated(50,-20);
        textElipse = textElipse.translated(rect().center() - item->pos());
        qDebug() << "center" << rect().center();
        //path = path.translated(QPoint(100,100));
        qDebug() << "text elipse" << textElipse;
        path.arcMoveTo(textElipse,m_rotation + (anglePerItem * count));
        path.arcTo(textElipse, m_rotation + (anglePerItem * count), anglePerItem);
        item->setPath(path);


        count++;
    }
}

void RadialSegment::addItem(RadialMenuItem* item){
    m_items << item;
    item->connect(item, SIGNAL(itemHovered()), this, SLOT(itemHovered()));
    item->connect(item, SIGNAL(itemSelected()), this, SLOT(itemSelected()));

    item->setParent(this);
    item->show();
    item->setFont(m_style.tick.font);
    relayoutItems();
}

void RadialSegment::removeItem(RadialMenuItem * ){
    /* not implemented yet */
}

void RadialSegment::mousePressEvent(QMouseEvent *event)
{
    relayoutItems();

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

QRectF RadialSegment::innerElipse() {
    return QRectF(-m_radius    + m_style.width,
                  -m_radius    + m_style.width,
                  (2*m_radius) - (2*m_style.width),
                  (2*m_radius) - (2*m_style.width));
}

QRectF RadialSegment::outerElipse() {
    return QRectF(-m_radius, -m_radius,
                  (2*m_radius), (2*m_radius));
}

QPainterPath RadialSegment::backgroundPath(){
    QPainterPath path;

    path.arcMoveTo(innerElipse(), m_rotation);
    path.arcTo(outerElipse(), m_rotation, m_angle);
    path.arcTo(innerElipse(), m_rotation + m_angle, - m_angle);
    path.closeSubpath();

    return path;
}

void RadialSegment::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    //painter.fillRect(this->rect(), Qt::green);


    painter.translate(width() / 2, height() / 2);
    painter.setPen(m_style.pen);
    painter.setBrush(m_style.brush);
    painter.setFont(m_style.tick.font);

    QPainterPath path = backgroundPath();
    painter.drawPath(path);


    painter.setPen(QPen(Qt::green, 5));

    foreach(RadialMenuItem * item, m_items){
        //painter.drawPath(item->getPath());
    }

}

QRegion RadialSegment::recomputeMask(){
    QRect outer(0, 0, m_radius*2, m_radius*2);
    QRegion outerRegion(outer, QRegion::Ellipse);
    QRegion innerRegion(outer.adjusted(m_style.width, m_style.width,
                                       -m_style.width, -m_style.width), QRegion::Ellipse);
    QRegion maskedRegion(outerRegion.subtract(innerRegion));
    //setMask(maskedRegion);
    Q_EMIT maskComputed(maskedRegion);
    return maskedRegion;
}

void RadialSegment::resizeEvent(QResizeEvent * /* event */)
{
    recomputeMask();
}

QSize RadialSegment::sizeHint() const
{
    return QSize(ceil(m_radius)*2, ceil(m_radius)*2);
}

const RadialMenuItem* RadialSegment::itemAt(QPoint const& p) const{
    QWidget * pick = childAt(p);
    if(!pick) return NULL;

    if (RadialMenuItem * segment = qobject_cast<RadialMenuItem*>(pick)){
        return segment;
    }
    return NULL;
}
