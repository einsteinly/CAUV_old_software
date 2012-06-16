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
        font::Verdana_8pt,
        10
    };
    return style;
}

const liquid::magma::RadialSegmentStyle& Default_RadialSegmentStyle() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::magma::RadialSegmentStyle style = {
        Default_RadialTickStyle(),
        Default_RadialTickStyle(),
        QPen(Qt::black, 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        60
    };
    return style;
}

const liquid::magma::RadialMenuStyle& Default_RadialMenuStyle() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::magma::RadialMenuStyle style = {
        Default_RadialSegmentStyle(),
        QPen(Qt::black, 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        0,
        5
    };
    return style;
}


}
}
