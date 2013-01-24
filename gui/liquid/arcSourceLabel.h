/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
