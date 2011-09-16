#ifndef __CAUV_GUI_ELEMENT_STYLE_H__
#define __CAUV_GUI_ELEMENT_STYLE_H__

#include <QColor>

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

const static ArcStyle Image_Arc_Style = {{
        QColor(255,255,255,235), 12,
        QColor(255,255,255,255), 6, 
        QColor(186,255,152,255),
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
        QColor(235,235,235,255), 6,
        QColor(186,255,152,255),        
        1, 12, 2, 16
    },{
        QColor(0,0,0,0), 12,
        QColor(0,0,0,128), 8,
        QColor(0,0,0,128),
        0, 14, 4, 17
    }
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_ELEMENT_STYLE_H__

