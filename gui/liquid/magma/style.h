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
    };

    struct RadialMenuStyle {
        RadialSegmentStyle segment;
        QPen   pen;
        QBrush brush;
        float spacing;
        float centreSpace;
    };

    const liquid::magma::RadialMenuStyle    &Default_RadialMenuStyle();
    const liquid::magma::RadialSegmentStyle &Default_RadialSegmentStyle();
    const liquid::magma::RadialTickStyle    &Default_RadialTickStyle();

} // namespace magma
} // namespace liquid

#endif // __LIQUID_MAGMA_STYLE_H__
