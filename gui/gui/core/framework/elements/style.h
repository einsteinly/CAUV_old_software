#ifndef __CAUV_GUI_ELEMENT_STYLE_H__
#define __CAUV_GUI_ELEMENT_STYLE_H__

#include <QColor>
#include <QPen>
#include <QBrush>

namespace cauv{
namespace gui{

struct ArcStyle{
    struct SingleArcStyle{
        QColor start_col;
        qreal start_length;
        
        QColor col;
        qreal thickness;

        QColor tip_hl_col;

        qreal end_base_offset;
        qreal end_base_thickness;
        qreal end_tip_thickness;
        qreal end_length;
    };
    SingleArcStyle front;
    SingleArcStyle back;
};

struct NodeStyle{
    QPen pen;
    QBrush brush;

    qreal tl_radius;
    qreal bl_radius;

    qreal header_height;

    QPen item_separator_pen;

    qreal in_socket_cutout_base;
    qreal in_socket_cutout_tip;
    qreal in_socket_cutout_depth;

    struct Input{
        struct Geometry{
            qreal cutout_base;
            qreal cutout_tip;
            qreal cutout_depth;
        };
        struct Style{
            QPen pen;
            QBrush brush;
        };

        Geometry Required;
        Geometry Optional;

        Style Param;
        Style Image;

    } InputStyle;
};

const static ArcStyle Image_Arc_Style = {{
        QColor(255,255,255,235), 12,
        QColor(255,255,255), 6, 
        QColor(186,255,152),
        1, 12, 2, 16
    },{
        QColor(0,0,0,0), 12,
        QColor(0,0,0,128), 8,
        QColor(0,0,0,128),
        0, 14, 4, 17
    }
};

const static ArcStyle Param_Arc_Style = {{
        QColor(235,235,235,235), 12,
        QColor(235,235,235), 6,
        QColor(186,255,152),        
        1, 12, 2, 16
    },{
        QColor(0,0,0,0), 12,
        QColor(0,0,0,128), 8,
        QColor(0,0,0,128),
        0, 14, 4, 17
    }
};

const static qreal Cut_S = (14/16.0);
const static NodeStyle Node_Style = {
    QPen(QBrush(QColor(0,0,0,128)), 1, Qt::SolidLine, Qt::FlatCap),
    QBrush(QColor(243,243,243)),
    24, 24,
    30,
    QPen(QColor(255,255,255)),
    14*Cut_S, 4*Cut_S, 16*Cut_S,

    /*InputStyle*/ {
        {9*Cut_S, 1*Cut_S, 12*Cut_S},
        {9*Cut_S, 1*Cut_S, 0*Cut_S},
        {QPen(QBrush(QColor(128,128,128)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(235,235,235))},
        {QPen(QBrush(QColor(160,160,160)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(255,255,255))}
        /*{QPen(QBrush(QColor(88,126,168)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(151,200,255))},
        {QPen(QBrush(QColor(134,102,154)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(208,181,225))}*/
    }
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_ELEMENT_STYLE_H__

