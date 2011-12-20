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

#ifndef CAUV_STYLE_H
#define CAUV_STYLE_H

#if defined _WIN32 || defined _WIN64
    #include <QWindowsXPStyle>
    #define BASESTYLE QWindowsXPStyle
#elif __APPLE__
    #include <QMacStyle>
    #define BASESTYLE QMacStyle
    //#include <QMotifStyle>
    //#define BASESTYLE QMotifStyle
#else
    #include <QPlastiqueStyle>
    #define BASESTYLE QPlastiqueStyle
#endif

#include <QStyleOptionProgressBarV2>


namespace cauv {
    namespace gui {

    namespace CauvStyleOptions {
        enum e{
            StyleOptionNeutralSpinBox = QStyleOption::SO_CustomBase + 1,
            StyleOptionGraphingSpinBox
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


    class StyleOptionGraphingSpinBox : public QStyleOptionSpinBox {
    public:
        enum StyleOptionType { Type = CauvStyleOptions::StyleOptionGraphingSpinBox };
        StyleOptionGraphingSpinBox() : QStyleOptionSpinBox(), samples() {
                type = CauvStyleOptions::StyleOptionGraphingSpinBox;
        }

        int maximum;
        int minimum;
        QList<int> samples;
    };

    class CauvStyle : public BASESTYLE
    {
        Q_OBJECT

    public:
        CauvStyle();

        QRect subControlRect ( ComplexControl control, const QStyleOptionComplex * option,
                               SubControl subControl, const QWidget * widget = 0 ) const;

        void drawControl(ControlElement control, const QStyleOption *option,
                         QPainter *painter, const QWidget *widget) const;

        void drawComplexControl(ComplexControl control, const QStyleOptionComplex *option,
                                QPainter *painter, const QWidget *widget) const;
    };


    } // namespace gui
} // namespace cauv


#endif // CAUV_STYLE_H
