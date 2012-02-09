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

#include "view.h"
#include "layout.h" // temporary, see keyPressEvent

#include <QGraphicsProxyWidget>

#include <QApplication>

#include <QGesture>

#include <QDebug>

#include <debug/cauv_debug.h>

#include <common/cauv_utils.h>

using namespace liquid;


class ZoomFilter : public QObject {
    bool eventFilter(QObject *object, QEvent *event)
    {

        if (event->type() == QEvent::KeyRelease) {
            QKeyEvent *keyEvent = dynamic_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape) {
                if(LiquidView * vis = dynamic_cast<LiquidView *>(object)){
                    vis->resetMatrix();
                    vis->centerOn(0,0);
                }
                // let the event propage
                return false;
            }
        }

        return false;
    }
};


LiquidView::LiquidView(QWidget * parent) : QGraphicsView(parent),
    m_scaleFactor(1.25), m_minScale(0.1), m_maxScale(1)
{
    this->setAcceptDrops(true);

    installEventFilter(new ZoomFilter());

    setAttribute(Qt::WA_AcceptTouchEvents);
    grabGesture(Qt::PinchGesture);

    setDragMode(QGraphicsView::ScrollHandDrag);

    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setTransformationAnchor(LiquidView::AnchorViewCenter);
    
    centerOn(0,0);
    
    setRenderHints(QPainter::Antialiasing |
                   QPainter::TextAntialiasing |
                   QPainter::SmoothPixmapTransform);
}

bool LiquidView::event(QEvent *event)
{
    if (event->type() == QEvent::Gesture)
        return gestureEvent(static_cast<QGestureEvent*>(event));
    return QGraphicsView::event(event);
}

bool LiquidView::gestureEvent(QGestureEvent *event)
{
    if (QGesture *gesture = event->gesture(Qt::PinchGesture)){
        QPinchGesture * pinch = static_cast<QPinchGesture *>(gesture);
        scaleAround(pinch->startCenterPoint(), ((pinch->scaleFactor()-1)*0.1)+1);
    }
    return true;
}



void LiquidView::scaleAround(QPointF point, qreal scaleFactor){

    if((scaleFactor > 1) && (transform().m11() > maxScale())) return;
    if((scaleFactor < 1) && (transform().m11() < minScale())) return;

    //QPoint viewportCenter = QPoint(rect().width()>>1, rect().height()>>1);
    //qDebug() << "viewportCenter" << viewportCenter;
    //QPointF screenCenter = mapToScene(viewportCenter);
    //qDebug() << "screenCenter" << screenCenter;


    //Get the position of the mouse before scaling, in scene coords
    //QPointF pointBeforeScale(mapToScene(point.toPoint()));
    //qDebug() << "pointBeforeScale" << pointBeforeScale;

    QTransform t = transform();
    float scale = t.m11() * scaleFactor;
    scale = clamp(minScale(), scale, maxScale());
    QTransform newT(scale, t.m12(), t.m13(),
                  t.m21(), scale, t.m23(),
                  t.m31(), t.m32(), t.m33());

    setTransform(newT);

    //Get the position after scaling, in scene coords
    //QPointF pointAfterScale(mapToScene(point.toPoint()));
    //qDebug() << "pointAfterScale" << pointAfterScale;


    //Get the offset of how the screen moved
    //QPointF offset = pointAfterScale - pointBeforeScale;
    //qDebug() << "offset" << offset;

    //QPointF newCenter = screenCenter - offset;
    //qDebug() << "newCenter" << newCenter;
    //centerOn(newCenter);
}



void LiquidView::wheelEvent(QWheelEvent *event){

    if(!(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
        return QGraphicsView::wheelEvent(event);
    } else {
        scaleAround(event->pos(), (((float)event->delta())*0.005)+1);
        event->accept();
        QGraphicsView::wheelEvent(event);
    }
}

void LiquidView::keyPressEvent(QKeyEvent *event){
    // temporary debug stuff: expect some sort of global hotkey system like
    // OverKey, but better
    switch(event->key()){
        case Qt::Key_L:
            LayoutItems::updateLayout(scene());
            return;
        default:
            QGraphicsView::keyPressEvent(event);
            return;
    }
}


float LiquidView::scaleFactor(){
    return m_scaleFactor;
}

void LiquidView::setScaleFactor(float scaleFactor){
    m_scaleFactor = scaleFactor;
}

float LiquidView::minScale(){
    return m_minScale;
}

void LiquidView::setMinScale(float scale){
    m_minScale = scale;
}

float LiquidView::maxScale(){
    return m_maxScale;
}

void LiquidView::setMaxScale(float scale){
    m_maxScale = scale;
}

