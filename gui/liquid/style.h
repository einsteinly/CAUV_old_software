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

    // provided by RequiresCutout subclasses
    struct CutoutStyle{
        struct CutoutGeometry{
            qreal cutout_base;
            qreal cutout_tip;
            qreal cutout_depth;
            qreal y_offset;
        };
        
        struct Style{
            QPen pen;
            QBrush brush;
        };
    
        CutoutGeometry main_cutout;
        CutoutGeometry second_cutout;
        Style style;
    };
    
    // provided by whatever constructs nodes
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
        
        // NB: colour is ignored - widget text is drawn by re-styled QLabel
        // Widgets, so style comes from CauvStyle
        TextStyle text;
    };

} // namespace liquid

#endif // __LIQUID_STYLE_H__
