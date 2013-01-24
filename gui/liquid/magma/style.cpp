/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "style.h"
#include <QPalette>
#include <QWidget>
#include <QPoint>

namespace liquid{
namespace magma{

namespace font{
const static QFont Verdana_12pt = QFont("Verdana", 12, 1);
const static QFont Verdana_10pt = QFont("Verdana", 10, 1);
const static QFont Verdana_8pt  = QFont("Verdana", 8, 1);
} // namespace font

const liquid::magma::RadialTickStyle& Default_RadialTickStyle() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::magma::RadialTickStyle style = {
        QPolygon() << QPoint(0,0) << QPoint(0, 1) << QPoint(1,1) << QPoint(1, 0),
        QPen(Qt::black, 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        font::Verdana_12pt,
        10
    };
    return style;
}

const liquid::magma::RadialSegmentStyle& Default_RadialSegmentStyle() {
    const static liquid::magma::RadialSegmentStyle style = {
        Default_RadialTickStyle(), // ticks
        Default_RadialTickStyle(), // source tick
        QPen(QColor(168, 173, 183, 210), 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(QColor(238, 243, 253, 230)),
        30, // width pixels
        30, // minAnglePerItem degrees
        340 // maxAngle degrees
    };
    return style;
}

const liquid::magma::RadialMenuStyle& Default_RadialMenuStyle() {
    const static liquid::magma::RadialMenuStyle style = {
        Default_RadialSegmentStyle(),
        5,  // spacing pixels
        30  // centreSpace pixels
    };
    return style;
}


}
}
