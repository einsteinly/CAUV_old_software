#ifndef __LIQUID_VIEW_H__
#define __LIQUID_VIEW_H__

#include <QGraphicsView>
#include <QWheelEvent>

namespace liquid {

class LiquidView : public QGraphicsView
{
public:
    LiquidView(QWidget * parent = NULL);

    virtual float scaleFactor();
    virtual void setScaleFactor(float scaleFactor);            

    void wheelEvent(QWheelEvent *event);

protected:
    float m_scaleFactor;
    QGraphicsScene * m_previousScene;
};

} // namespace liquid

#endif // __LIQUID_VIEW_H__
