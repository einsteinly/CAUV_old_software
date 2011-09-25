#ifndef __LIQUID_STYLE_H__
#define __LIQUID_STYLE_H__

#include <QColor>
#include <QPen>
#include <QBrush>
#include <QFont>

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

    struct TextStyle{
        QPen   pen;
        QBrush brush;
        QFont  font;
    };

    struct NodeStyle{
        QPen pen;
        QBrush brush;

        qreal tl_radius;
        qreal bl_radius;

        struct Header_Style{
            qreal height;
            QPen pen;
            QBrush brush;
            TextStyle title;
            TextStyle info;
        } header;

        QPen item_separator_pen;
        QPen resize_handle_pen;

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

} // namespace liquid

#endif // __LIQUID_STYLE_H__
