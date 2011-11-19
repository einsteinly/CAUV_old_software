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

namespace liquid {

class LiquidView : public QGraphicsView
{
public:
    LiquidView(QWidget * parent = NULL);

    virtual float scaleFactor();
    virtual void setScaleFactor(float scaleFactor);

    virtual float minScale();
    virtual void setMinScale(float scale);

    virtual float maxScale();
    virtual void setMaxScale(float scale);

    void wheelEvent(QWheelEvent *event);

protected:
    float m_scaleFactor;
    float m_minScale;
    float m_maxScale;
    QGraphicsScene * m_previousScene;
};

} // namespace liquid

#endif // __LIQUID_VIEW_H__
