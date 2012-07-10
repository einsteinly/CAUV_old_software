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
