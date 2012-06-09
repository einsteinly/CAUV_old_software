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

#ifndef __CAUV_GUI_ELEMENT_STYLE_H__
#define __CAUV_GUI_ELEMENT_STYLE_H__

#include <liquid/style.h>

namespace cauv{
namespace gui{

const static liquid::ArcStyle Image_Arc_Style = {{
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

const static liquid::ArcStyle Param_Arc_Style = {{
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


const static liquid::CutoutStyle Required_Image_Input = {
    {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
    {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
    {QPen(QBrush(QColor(160,160,160)), 1, Qt::SolidLine, Qt::FlatCap),
     QBrush(QColor(255,255,255))}
};

const static liquid::CutoutStyle Required_Param_Input = {
    {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
    {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
    {QPen(QBrush(QColor(128,128,128)), 1, Qt::SolidLine, Qt::FlatCap),
     QBrush(QColor(235,235,235))}
};

const static liquid::CutoutStyle Optional_Image_Input = {
    {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
    {9*Cut_S, 1*Cut_S, 0*Cut_S, 0},
    {QPen(QBrush(QColor(160,160,160)), 1, Qt::SolidLine, Qt::FlatCap),
     QBrush(QColor(255,255,255))}
};

const static liquid::CutoutStyle Optional_Param_Input = {
    {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
    {9*Cut_S, 1*Cut_S, 0*Cut_S, 0},
    {QPen(QBrush(QColor(128,128,128)), 1, Qt::SolidLine, Qt::FlatCap),
     QBrush(QColor(235,235,235))}
};

namespace font{
const static QFont Verdana_12pt = QFont("Verdana", 12, 1);
const static QFont Verdana_10pt = QFont("Verdana", 10, 1);
const static QFont Verdana_8pt  = QFont("Verdana", 8, 1);
} // namespace font

const static liquid::NodeStyle F_Node_Style = {
    QPen(QBrush(QColor(0,0,0,128)), 1, Qt::SolidLine, Qt::FlatCap),
    QBrush(QColor(243,243,243)),
    24, 24, 0, 0, {
        30,
        QPen(Qt::NoPen),
        QBrush(QColor(0,0,0,160)), {
            //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
            QPen(Qt::NoPen),
            QBrush(QColor(255,255,255)),
            font::Verdana_12pt
        },{
            QPen(Qt::NoPen),
            //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
            QBrush(QColor(255,255,255)),
            font::Verdana_8pt 
        }
    },
    QPen(QColor(255,255,255)),
    QPen(QBrush(QColor(190,190,190)), 2, Qt::SolidLine, Qt::RoundCap),
    {
        QPen(Qt::NoPen),
        QBrush(QColor(12,12,12)),
        font::Verdana_10pt 
    }
};

const static liquid::NodeStyle AI_Node_Style = {
    QPen(QBrush(QColor(0,0,0,128)), 1, Qt::SolidLine, Qt::FlatCap),
    QBrush(QColor(243,243,243)),
    24, 24, 0, 0, {
        30,
        QPen(Qt::NoPen),
        QBrush(QColor(69,121,173,160)), {
            QPen(Qt::NoPen),
            QBrush(QColor(255,255,255)),
            font::Verdana_12pt
        },{
            QPen(Qt::NoPen),
            QBrush(QColor(255,255,255)),
            font::Verdana_8pt 
        }
    },
    QPen(QColor(255,255,255)),
    QPen(QBrush(QColor(190,190,190)), 2, Qt::SolidLine, Qt::RoundCap),
    {
        QPen(Qt::NoPen),
        QBrush(QColor(12,12,12)),
        font::Verdana_10pt 
    }
};

const static liquid::NodeStyle Graph_Node_Style = {
    QPen(QBrush(QColor(0,0,0,128)), 1, Qt::SolidLine, Qt::FlatCap),
    QBrush(QColor(243,243,243)),
    24, 24, 0, 0, {
        30,
        QPen(Qt::NoPen),
        QBrush(QColor(255,255,255,0)), {
            QPen(Qt::NoPen),
            QBrush(QColor(0,0,0)),
            font::Verdana_12pt
        },{
            QPen(Qt::NoPen),
            QBrush(QColor(0,0,0)),
            font::Verdana_8pt 
        }
    },
    QPen(QColor(255,255,255)),
    QPen(QBrush(QColor(190,190,190)), 2, Qt::SolidLine, Qt::RoundCap),
    {
        QPen(Qt::NoPen),
        QBrush(QColor(12,12,12)),
        font::Verdana_10pt 
    }
};


} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_ELEMENT_STYLE_H__

