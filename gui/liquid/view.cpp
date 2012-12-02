/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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
#include <QPixmapCache>

#include <debug/cauv_debug.h>

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
    setAcceptDrops(true);
    setCacheMode(QGraphicsView::CacheBackground);

    installEventFilter(new ZoomFilter());

    setAttribute(Qt::WA_AcceptTouchEvents);
    grabGesture(Qt::PinchGesture);
    grabGesture(Qt::PanGesture);

    setDragMode(QGraphicsView::ScrollHandDrag);

    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    //setTransformationAnchor(QGraphicsView::NoAnchor);
    setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    
    centerOn(0,0);
    
    const int cache_kbytes = 1024 * 64; // 64 MB, default is 10
    if(QPixmapCache::cacheLimit() < cache_kbytes) 
        QPixmapCache::setCacheLimit(cache_kbytes); 
    
    setRenderHints(QPainter::Antialiasing |
                   QPainter::TextAntialiasing |
                   QPainter::SmoothPixmapTransform);
}

LiquidView::~LiquidView()
{
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
        scaleAround(pinch->startCenterPoint().toPoint(), ((pinch->scaleFactor()-1)*0.1)+1);
    }
    return true;
}



void LiquidView::scaleAround(QPoint point, qreal scaleFactor){

    if(scaleFactor * transform().m11() >= maxScale()){
        scaleFactor = maxScale() / transform().m11();
    }
    if(scaleFactor * transform().m11() < minScale()){
        scaleFactor = minScale() / transform().m11();
    }

    QPointF around = mapToScene(point);

    qDebug() << around;

    /*

    QMatrix translation(1, 0, 0, 1, around.x(), around.y());
    QMatrix reverseTranslation(1, 0, 0, 1, -around.x(), -around.y());

    float scalar = transform().m11() * scaleFactor;
    scalar = clamp(minScale(), scalar, maxScale());
    QMatrix scale(scalar, 0, 0, scalar, 1, 1);

    // some weird behaviour here (on mac at least)
    // the initial scaling work for a bit, then it goes mental
    // not sure why yet...
    //setTransform(QTransform((reverseTranslation * scale) * translation));
    setTransform(QTransform(scale));*/
    
    scale(scaleFactor, scaleFactor);
}


#ifdef QT_PROFILE_GRAPHICSSCENE
void LiquidView::displayUpdateRects(){
    m_display_rects = getRectsUpdated();
    clearRectsUpdated();
}
#endif // def QT_PROFILE_GRAPHICSSCENE


std::ostream& operator<<(std::ostream& os, QStringList const& sl){
    os << "QStringList:";
    foreach(QString s, sl)
        os << '\n' << s.toUtf8().data();
    return os;
}

void LiquidView::wheelEvent(QWheelEvent *event){

    if(!(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
        return QGraphicsView::wheelEvent(event);
    } else {
        scaleAround(event->pos(), (((float)event->delta())*0.005)+1);
        event->accept();
        //QGraphicsView::wheelEvent(event);
    }
}

void LiquidView::keyPressEvent(QKeyEvent *event){
    // temporary debug stuff: expect some sort of global hotkey system like
    // OverKey, but better
    QGraphicsView::keyPressEvent(event);
    if(!event->isAccepted()){
        switch(event->key()){
            case Qt::Key_L:
                event->accept();
                LayoutItems::updateLayout(scene());
                break;

        #ifdef QT_PROFILE_GRAPHICSSCENE
            case Qt::Key_P:
                event->accept();
                if(m_display_rects.size())
                    m_display_rects.clear();
                else
                    displayUpdateRects();
                break;
        #endif // def QT_PROFILE_GRAPHICSSCENE
            default:
                Q_EMIT keyPressed(event->key(), event->modifiers());
                event->accept();
                break;
        }
    }
    event->ignore();
}


#ifdef QT_PROFILE_GRAPHICSSCENE
void LiquidView::drawForeground(QPainter* painter, const QRectF& rect){
    
    if(m_display_rects.size() && getRectsUpdated().size() > 10){
        displayUpdateRects();
    }

    painter->setPen(QPen(QColor(160,20,20,24)));
    painter->setBrush(QBrush(QColor(180,30,30,12)));

    foreach(QRectF const& r, m_display_rects)
        painter->drawPolygon(mapToScene(r.toRect()));
    
}
#endif // def QT_PROFILE_GRAPHICSSCENE


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

