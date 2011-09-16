#ifndef __CAUV_GUI_ELEMENT_STYLE_H__
#define __CAUV_GUI_ELEMENT_STYLE_H__

#include <QColor>
#include <QPen>
#include <QBrush>

namespace cauv{
namespace gui{

struct ArcStyle{
    struct _SingleArcStyle{
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
    _SingleArcStyle front;
    _SingleArcStyle back;
};

struct NodeStyle{
    QPen pen;
    QBrush brush;

    qreal tl_radius;
    qreal bl_radius;

    QPen item_separator_pen;

    qreal in_socket_cutout_base;
    qreal in_socket_cutout_tip;
    qreal in_socket_cutout_depth;
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

const static NodeStyle Node_Style = {
    QPen(QBrush(QColor(0,0,0,128)), 1, Qt::SolidLine, Qt::FlatCap),
    QBrush(QColor(243,243,243)),
    24, 24,
    QPen(QColor(255,255,255)),
    14*(15/16.0), 4*(15/16.0), 16*(15/16.0)
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_ELEMENT_STYLE_H__

