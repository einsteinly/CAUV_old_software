#ifndef __LIQUID_ARC_STYLE_H__
#define __LIQUID_ARC_STYLE_H__

namespace liquid {

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

} // namespace liquid

#endif // __LIQUID_ARC_STYLE_H__
