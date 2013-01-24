/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */



#ifndef __LIQUID_MAGMA_RADIAL_MENU_ITEM_H__
#define __LIQUID_MAGMA_RADIAL_MENU_ITEM_H__

#include <QtGui>

#include "labelPath.h"

namespace liquid {
namespace magma {

class RadialSegment;

class RadialMenuItem : public LabelPath {
    Q_OBJECT

public:
    RadialMenuItem(QModelIndex const& index,
                   RadialSegment * segment,
                   Qt::WindowFlags f = 0
            );

    RadialMenuItem(QModelIndex const& index,
                   RadialSegment * segment,
                   const QString& text,
                   Qt::WindowFlags f = 0
            );

    void init();
    void enterEvent(QEvent *);
    void leaveEvent(QEvent *);
    void mouseReleaseEvent(QMouseEvent *ev);
    QModelIndex index() const;
    RadialSegment * segment() const;


Q_SIGNALS:
    void itemHovered();
    void itemSelected();

protected:
    const QModelIndex m_index;
    RadialSegment * m_segment;
    QTimer m_hoverTimer;
};

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_RADIAL_MENU_ITEM_H__
