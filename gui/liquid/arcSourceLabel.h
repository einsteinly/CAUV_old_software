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

#ifndef __LIQUID_ARC_SOURCE_LABEL_H__
#define __LIQUID_ARC_SOURCE_LABEL_H__

#include <QGraphicsProxyWidget>
#include <QGraphicsWidget>

#include "arcSource.h"

class QGraphicsLinearLayout;

namespace liquid {

class ArcSourceLabel: public QGraphicsWidget {
public:
    AbstractArcSource* source() const;

    ArcSourceLabel(ArcSource *arc_source,
                   LiquidNode *node,
                   const QString &id);
    virtual ~ArcSourceLabel();

    // QGraphicsItem:
    virtual void paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget = 0);

    // Safe access to the layout:
    //QGraphicsLinearLayout* vLayout() const;
    QGraphicsLinearLayout* hLayout() const;

protected:
    liquid::ArcSource* m_arc_source;
    QGraphicsProxyWidget* m_text;

    QGraphicsLinearLayout* m_hlayout;
};

} // namespace liquid

#endif //  __LIQUID_ARC_SOURCE_LABEL_H__
