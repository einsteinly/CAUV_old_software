/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_MAGMA_STYLE_H__
#define __LIQUID_MAGMA_STYLE_H__

#include <QColor>
#include <QPen>
#include <QBrush>
#include <QFont>
#include <QPolygon>

namespace liquid {
namespace magma {

    struct RadialTickStyle {
        QPolygon shape;
        QPen   pen;
        QBrush brush;
        QFont  font;
        float length;
    };

    struct RadialSegmentStyle {
        RadialTickStyle tick;
        RadialTickStyle source;
        QPen   pen;
        QBrush brush;
        float width;
        float preferredAnglePerItem;
        float maxAngle;
    };

    struct RadialMenuStyle {
        RadialSegmentStyle segment;
        float spacing;
        float centreSpace;
    };

    const liquid::magma::RadialMenuStyle    &Default_RadialMenuStyle();
    const liquid::magma::RadialSegmentStyle &Default_RadialSegmentStyle();
    const liquid::magma::RadialTickStyle    &Default_RadialTickStyle();

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_STYLE_H__
