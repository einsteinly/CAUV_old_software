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

#ifndef __LIQUID_VIEW_H__
#define __LIQUID_VIEW_H__

#include <QGraphicsView>
#include <QWheelEvent>
#include <QGestureEvent>

namespace liquid {

class LiquidView : public QGraphicsView
{
    Q_OBJECT
public:
    LiquidView(QWidget * parent = NULL);
    virtual ~LiquidView();

    virtual float scaleFactor();
    virtual void setScaleFactor(float scaleFactor);

    virtual float minScale();
    virtual void setMinScale(float scale);

    virtual float maxScale();
    virtual void setMaxScale(float scale);

    void scaleAround(QPoint point, qreal scaleFactor);
    
    #ifdef QT_PROFILE_GRAPHICSSCENE
    void displayUpdateRects();
    #endif // def QT_PROFILE_GRAPHICSSCENE
    
protected:
    virtual bool event(QEvent *event);
    virtual bool gestureEvent(QGestureEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    
#ifdef QT_PROFILE_GRAPHICSSCENE
    void drawForeground(QPainter* painter, const QRectF& rect);
#endif // def QT_PROFILE_GRAPHICSSCENE

Q_SIGNALS:
    void keyPressed(int key, Qt::KeyboardModifiers);

protected:
    float m_scaleFactor;
    float m_minScale;
    float m_maxScale;
    QGraphicsScene * m_previousScene;

    #ifdef QT_PROFILE_GRAPHICSSCENE
    QList<QRectF> m_display_rects;
    #endif // def QT_PROFILE_GRAPHICSSCENE

};

} // namespace liquid

#endif // __LIQUID_VIEW_H__
