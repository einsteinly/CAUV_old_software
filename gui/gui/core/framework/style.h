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

#ifndef __CAUV_STYLE_H__
#define __CAUV_STYLE_H__

#define USE_QMacStyle

#if defined _WIN32 || defined _WIN64
    #include <QWindowsXPStyle>
    #define BASESTYLE QWindowsXPStyle
#elif __APPLE__
    #ifdef USE_QMacStyle
        #include <QMacStyle>
        #define BASESTYLE QMacStyle
    #else
        #include <QMotifStyle> 
        #define BASESTYLE QMotifStyle
    #endif
#else
    #include <QPlastiqueStyle>
    #define BASESTYLE QPlastiqueStyle
#endif

#include <QStyleOptionSpinBox>

#include <common/cauv_utils.h>

namespace cauv {
namespace gui {

template<class T>
QColor cauvColorMap(T min, T max, T value, bool inverted = true, int hueRange = 100, QColor startingColor = QColor::fromHsl(0, 160, 200)){

    T scalar = pivot(min, 0, max, value);

    int hue = hueRange * scalar;
    if (inverted)
        hue = hueRange - hue;

    startingColor.setHsl(startingColor.hue() + hue, startingColor.saturation(), startingColor.lightness());
    return startingColor;
}


namespace CauvStyleOptions {
enum e{
    StyleOptionNeutralSpinBox = QStyleOption::SO_CustomBase + 1,
    StyleOptionGraphingSpinBox,
    StyleOptionOnOff
};
}

class StyleOptionNeutralSpinBox : public QStyleOptionSpinBox {
public:
    enum StyleOptionType { Type = CauvStyleOptions::StyleOptionNeutralSpinBox };
    StyleOptionNeutralSpinBox() : QStyleOptionSpinBox(), level(0), invertColours(true){
        type = CauvStyleOptions::StyleOptionNeutralSpinBox;
    }

    float level;
    bool invertColours;
};

class StyleOptionOnOff : public QStyleOptionButton {
public:
    enum StyleOptionType { Type = CauvStyleOptions::StyleOptionOnOff };
    StyleOptionOnOff() : QStyleOptionButton(), position(0), marked(true) {
        type = CauvStyleOptions::StyleOptionOnOff;
    }

    float position;
    bool marked;
};


class StyleOptionGraphingSpinBox : public QStyleOptionSpinBox {
public:
    enum StyleOptionType { Type = CauvStyleOptions::StyleOptionGraphingSpinBox };
    StyleOptionGraphingSpinBox() : QStyleOptionSpinBox(), samples() {
        type = CauvStyleOptions::StyleOptionGraphingSpinBox;
    }

    //!!! todo:
    // this is very inefficient
    // it should all be done as integer artihmetic
    // but this was quicker to implement to try it out
    QVariant maximum;
    QVariant minimum;
    QList<QVariant> samples;
};

class CauvStyle : public BASESTYLE
{
    Q_OBJECT

public:
    enum CauvControlElement {
        CE_Graph = QStyle::CE_CustomBase + 1
    };

    CauvStyle();

    QRect subControlRect ( ComplexControl control, const QStyleOptionComplex * option,
                           SubControl subControl, const QWidget * widget = 0 ) const;

    void drawControl(CauvControlElement control, const QStyleOption *option,
                     QPainter *painter, const QWidget *widget) const;

    void drawControl(ControlElement control, const QStyleOption *option,
                     QPainter *painter, const QWidget *widget) const;

    void drawComplexControl(ComplexControl control, const QStyleOptionComplex *option,
                            QPainter *painter, const QWidget *widget) const;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_STYLE_H__
