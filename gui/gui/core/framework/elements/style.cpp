#include "style.h"
#include <QPalette>
#include <QWidget>

namespace cauv{
namespace gui{

static QColor alpha(QColor c, int alpha) {
    c.setAlpha(alpha);
    return c;
}

namespace font{
const static QFont Verdana_12pt = QFont("Verdana", 12, 1);
const static QFont Verdana_10pt = QFont("Verdana", 10, 1);
const static QFont Verdana_8pt  = QFont("Verdana", 8, 1);
} // namespace font

const liquid::ArcStyle& Image_Arc_Style() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::ArcStyle style = {{
            alpha(p.color(QPalette::Window), 235), 12,
            p.color(QPalette::Window), 6, 
            QColor(186,255,152),
            1, 12, 2, 16
        },{
            QColor(0,0,0,0), 12,
            alpha(p.color(QPalette::WindowText), 128), 8,
            alpha(p.color(QPalette::WindowText), 128),
            0, 14, 4, 17
        }
    };
    return style;
}

const liquid::ArcStyle& Param_Arc_Style() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::ArcStyle style = {{
            alpha(p.color(QPalette::Window), 235), 12,
            p.color(QPalette::Window), 6, 
            QColor(186,255,152),
            1, 12, 2, 16
        },{
            QColor(0,0,0,0), 12,
            alpha(p.color(QPalette::WindowText), 128), 8,
            alpha(p.color(QPalette::WindowText), 128),
            0, 14, 4, 17
        }
    };
    return style;
}

const liquid::NodeStyle& F_Node_Style() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::NodeStyle style = {
        QPen(QBrush(alpha(p.color(QPalette::WindowText),128)), 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        24, 24, 0, 0, {
            30,
            QPen(Qt::NoPen),
            QBrush(alpha(p.color(QPalette::Shadow), 160)), {
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QPen(Qt::NoPen),
                QBrush(p.color(QPalette::HighlightedText)),
                font::Verdana_12pt
            },{
                QPen(Qt::NoPen),
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QBrush(p.color(QPalette::HighlightedText)),
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
    return style;
}

const liquid::NodeStyle& AI_Node_Style() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::NodeStyle style = {
        QPen(QBrush(alpha(p.color(QPalette::WindowText),128)), 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        24, 24, 0, 0, {
            30,
            QPen(Qt::NoPen),
            QBrush(QColor(69,121,173,160)), {
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QPen(Qt::NoPen),
                QBrush(p.color(QPalette::WindowText)),
                font::Verdana_12pt
            },{
                QPen(Qt::NoPen),
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QBrush(p.color(QPalette::WindowText)),
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
    return style;
}

const liquid::NodeStyle& Graph_Node_Style() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::NodeStyle style = {
        QPen(QBrush(alpha(p.color(QPalette::WindowText),128)), 1, Qt::SolidLine, Qt::FlatCap),
        QBrush(p.color(QPalette::Window)),
        24, 24, 0, 0, {
            30,
            QPen(Qt::NoPen),
            QBrush(QColor(0,0,0,0)), {
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QPen(Qt::NoPen),
                QBrush(p.color(QPalette::WindowText)),
                font::Verdana_12pt
            },{
                QPen(Qt::NoPen),
                //QPen(QBrush(QColor(0,0,0,64)), 1, Qt::SolidLine, Qt::FlatCap),
                QBrush(p.color(QPalette::WindowText)),
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
    return style;
}

const static qreal Cut_S = (14/16.0);

const liquid::CutoutStyle& Required_Image_Input() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::CutoutStyle style = {
        {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
        {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
        {QPen(QBrush(p.color(QPalette::Midlight)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(255,255,255))}
    };
    return style;
}

const liquid::CutoutStyle& Required_Param_Input() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::CutoutStyle style = {
        {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
        {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
        {QPen(QBrush(p.color(QPalette::Mid)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(QColor(255,255,255))}
    };
    return style;
}

const liquid::CutoutStyle& Optional_Image_Input() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::CutoutStyle style = {
        {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
        {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
        {QPen(QBrush(p.color(QPalette::Midlight)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(p.color(QPalette::Light))}
    };
    return style;
}

const liquid::CutoutStyle& Optional_Param_Input() {
    QWidget w;
    QPalette p = w.palette();
    const static liquid::CutoutStyle style = {
        {14*Cut_S, 4*Cut_S, 16*Cut_S, 0},
        {9*Cut_S, 1*Cut_S, 12*Cut_S, 0},
        {QPen(QBrush(p.color(QPalette::Mid)), 1, Qt::SolidLine, Qt::FlatCap),
         QBrush(p.color(QPalette::Light))}
    };
    return style;
}
}
}
