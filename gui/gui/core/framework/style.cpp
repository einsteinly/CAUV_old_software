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

#include "style.h"

#include <debug/cauv_debug.h>

#include <widgets/neutralspinbox.h>

#include <QtGui>

using namespace cauv;
using namespace cauv::gui;


#if defined _WIN32 || defined _WIN64
#include <QWindowsXPStyle>
#define BASESTYLE QWindowsXPStyle
#elif __APPLE__
#include <QMacStyle>
#define BASESTYLE QMacStyle
#else
#include <QPlastiqueStyle>
#define BASESTYLE QPlastiqueStyle
#endif


CauvStyle::CauvStyle() {
}

QRect CauvStyle::subControlRect ( ComplexControl control, const QStyleOptionComplex * option,
                                  SubControl subControl, const QWidget * widget) const {
    switch(subControl) {
    case QStyle::SC_SpinBoxUp: {
        QRect frame = BASESTYLE::subControlRect(control, option, SC_SpinBoxFrame, widget);
        QRect rect = BASESTYLE::subControlRect(control, option, subControl, widget);
        rect.setX((frame.x()+frame.width())-frame.height());
        rect.setWidth(frame.height());
        rect.setHeight(frame.height());

        return rect;
    }
    break;

    case QStyle::SC_SpinBoxDown: {
        QRect frame = BASESTYLE::subControlRect(control, option, SC_SpinBoxFrame, widget);
        QRect rect = BASESTYLE::subControlRect(control, option, subControl, widget);
        rect.setY(frame.y());
        rect.setX(frame.x());
        rect.setWidth(frame.height());
        rect.setHeight(frame.height());

        return rect;
    }
    break;

    case QStyle::SC_SpinBoxEditField: {
        QRect frame = BASESTYLE::subControlRect(control, option, SC_SpinBoxFrame, widget);
        QRect rect = BASESTYLE::subControlRect(control, option, subControl, widget);
        rect.setLeft(frame.height());
        rect.setRight(frame.right()-frame.height());
        return rect;
    }
    break;

    default: return BASESTYLE::subControlRect(control, option, subControl, widget);
    }
}

void CauvStyle::drawControl(ControlElement control, const QStyleOption *option,
                            QPainter *painter, const QWidget *widget) const{

    switch(control) {
    case CE_ProgressBar: {

        const QStyleOptionProgressBarV2 * progressOptions = static_cast<const QStyleOptionProgressBarV2 *>(option);
        const StyleOptionNeutralBar *neutralOptions = qstyleoption_cast<const StyleOptionNeutralBar *>(option);

        QStyleOptionProgressBarV2 outputOptions(*progressOptions);

        if ((neutralOptions) && (neutralOptions->type == CauvStyleOptions::StyleOptionNeutralBar)){

            float neutral = neutralOptions->neutral;

            if (progressOptions->progress < neutral){
                outputOptions.minimum = neutral;
                outputOptions.maximum = neutral + (neutral - progressOptions->minimum);
                outputOptions.progress = neutral + (neutral - progressOptions->progress);
            } else{
                outputOptions.minimum = neutral;
                outputOptions.maximum = progressOptions->maximum;
                outputOptions.progress = progressOptions->progress;
            }
        }

        int hueRange = 90;
        int range = outputOptions.maximum - outputOptions.minimum;
        if (range < 0) range = 0;

        float progress = 0;
        if(range != 0)
            progress = (float) outputOptions.progress / (float) range;

        int hue = hueRange - (hueRange * progress);

        QColor progressColor(QColor::fromHsl(hue, 160, 162));
        outputOptions.palette.setColor(QPalette::Highlight, progressColor);

        BASESTYLE::drawControl(control, &outputOptions, painter, widget);
    }
    break;

    default:
        BASESTYLE::drawControl(control, option, painter, widget);

    }
}

void CauvStyle::drawComplexControl(ComplexControl control, const QStyleOptionComplex *option, QPainter *painter, const QWidget *widget) const{
    switch (control) {
    case CC_SpinBox:
    {
        const QSpinBox * spin = qobject_cast<const QSpinBox*>(widget);
        if(spin){
            StyleOptionNeutralBar neutralOptions;
            neutralOptions.rect = option->rect;
            neutralOptions.direction = Qt::LeftToRight;
            neutralOptions.state = QStyle::State_Enabled;
            neutralOptions.minimum = spin->minimum();
            neutralOptions.neutral = spin->minimum();
            neutralOptions.maximum = spin->maximum();
            neutralOptions.progress = spin->value();
            neutralOptions.textVisible = false;

            // neutral spins have a neutral value to show where the default value is
            const NeutralSpinBox * neutralSpin = qobject_cast<const NeutralSpinBox*>(widget);
            if(neutralSpin) {
                neutralOptions.neutral = neutralSpin->getNeutral();
            }

            drawControl(CE_ProgressBar, &neutralOptions, painter, widget);

            painter->setOpacity(0.5);

            painter->setRenderHint(QPainter::Antialiasing);
            painter->setBrush(QBrush(QColor(200,200,200)));
            int buttonSize = option->rect.height()-12;
            painter->drawRoundedRect(QRect(6, 6, buttonSize, buttonSize), 10, 10);
            painter->drawRoundedRect(QRect(option->rect.right()-(6+buttonSize), 6, buttonSize, buttonSize), 10, 10);

            painter->setBrush(QBrush(Qt::gray));
            painter->setPen(QPen(QBrush(Qt::gray), 3.0));
            painter->drawRoundRect(QRect(10, 5 + (buttonSize/2), buttonSize-9, 2), 4, 4);
            painter->drawRoundRect(QRect(option->rect.right()-(10+buttonSize) + 8, 5 + (buttonSize/2), buttonSize-8, 2), 3, 3);
            painter->drawRoundRect(QRect(option->rect.right()-(buttonSize/2) - 7, 10, 2, buttonSize-8), 3, 3);

        }
    }
    break;
    default:
        BASESTYLE::drawComplexControl(control, option, painter, widget);
    }
}


