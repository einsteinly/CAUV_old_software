/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_ARC_SINK_LABEL_H__
#define __LIQUID_ARC_SINK_LABEL_H__

#include <QGraphicsProxyWidget>
#include <QGraphicsWidget>

#include "arcSink.h"

class QGraphicsLinearLayout;

namespace liquid {

class ArcSinkLabel: public QGraphicsWidget,
                    public RequiresCutout {
    public:
        AbstractArcSink* sink() const;

        ArcSinkLabel(ArcSink *arc_sink,
                     LiquidNode *node,
                     const QString &id);
        virtual ~ArcSinkLabel();

        // RequiresCutout:
        virtual QList<liquid::CutoutStyle> cutoutGeometry() const;

        // QGraphicsItem:
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget = 0);

        // Safe access to the layout:
        //QGraphicsLinearLayout* vLayout() const;
        QGraphicsLinearLayout* hLayout() const;

    protected:
        liquid::ArcSink* m_arc_sink;
        QGraphicsProxyWidget* m_text;

        QGraphicsLinearLayout* m_hlayout;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SINK_LABEL_H__
