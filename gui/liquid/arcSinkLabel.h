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
        QGraphicsLinearLayout* vLayout() const;

    protected:
        liquid::ArcSink* m_arc_sink;
        QGraphicsProxyWidget* m_text;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SINK_LABEL_H__
