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

#include <QtGui>
#include <QPainterPath>

#include <debug/cauv_debug.h>
#include <utility/rounding.h>

using namespace cauv;
using namespace cauv::gui;


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

void CauvStyle::drawControl(CauvControlElement control, const QStyleOption *option,
                            QPainter *painter, const QWidget *widget) const{
    drawControl((QStyle::ControlElement) control, option, painter, widget);
}

void CauvStyle::drawControl(ControlElement control, const QStyleOption *option,
                            QPainter *painter, const QWidget *widget) const{

    switch((CauvControlElement) control) {

    case CE_Graph: {

        const StyleOptionGraphingSpinBox *graphing = qstyleoption_cast<const StyleOptionGraphingSpinBox *>(option);
        if(graphing){
            painter->setRenderHint(QPainter::Antialiasing);
            QRect canvas = option->rect;
            canvas.setTop(canvas.top() + 5);
            canvas.setLeft(canvas.left() + 2);
            canvas.setRight(canvas.right() - 2);
            QList<QVariant> samples = graphing->samples;

            if(samples.empty()) return;

            float minimum = graphing->minimum.toFloat();
            float maximum = graphing->maximum.toFloat();

            float stepSize = ((float)canvas.width()) / (float)samples.size();
            float range = maximum - minimum;
            float heightScalar = ((float)canvas.height())/(float)range;
            float x = canvas.left();
            float yOffset = fabs(minimum) * heightScalar;
            float y = canvas.bottom() - (yOffset + (samples.first().toFloat() * heightScalar));

            painter->setPen(QPen(QColor(238, 238, 238)));
            painter->drawLine(canvas.left(), yOffset, canvas.right(), yOffset);

            QPainterPath path;
            path.moveTo(x, canvas.bottom() - yOffset);
            path.lineTo(x, y);
            path.setFillRule(Qt::WindingFill);
            x += stepSize;

            float lastSample = 0;
            foreach (QVariant sample, samples){
                float clampedSample = clamp(minimum, sample.toFloat(), maximum);

                float height = clampedSample * heightScalar;
                if((lastSample == 0) && (clampedSample == 0))
                    path.moveTo(x,  canvas.bottom() - (yOffset + height));
                else path.lineTo(x,  canvas.bottom() - (yOffset + height));

                lastSample = clampedSample;
                x += stepSize;
            }

            QColor outline = QColor(200, 200, 200);//cauvColorMap(minimum, maximum, samples.last().toFloat());
            QColor fill = QColor(230, 230, 230);//cauvColorMap(minimum, maximum, samples.last().toFloat(),
            //             true, 100, QColor::fromHsl(0,160,230));

            painter->strokePath(path, QPen(outline, 1));

            path.lineTo(canvas.right(), y);
            path.lineTo(canvas.right(), canvas.bottom() - yOffset);
            painter->fillPath(path, QBrush(fill));

            return;
        }
    }
    }

    switch(control) {

    case CE_FocusFrame:
        return;

    case CE_CheckBox: {

        if (const StyleOptionOnOff * onOffOption = qstyleoption_cast<const StyleOptionOnOff *>(option)) {

            painter->setRenderHint(QPainter::Antialiasing, true);
            QRect canvas = option->rect;
            canvas.setHeight(canvas.height()-4);
            canvas.setWidth(60);
            canvas.setX(canvas.x()+4);
            canvas.setY(canvas.y()+4);

            // draw frame
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::gray));
            painter->drawRoundedRect(canvas, 5, 5);

            // clear background
            QColor offColor(148, 148, 148);
            painter->setPen(Qt::NoPen);
            QLinearGradient bg(0, 20, 0, 0);
            bg.setSpread(QLinearGradient::ReflectSpread);
            bg.setColorAt(0, offColor);
            bg.setColorAt(1, offColor.darker(110));
            painter->setBrush(bg);
            painter->drawRoundedRect(canvas, 5, 5);



            QRect on = canvas;
            on.setWidth((canvas.width()/2) + 5);

            // draw frame
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::gray));
            painter->drawRoundedRect(on, 5, 5);

            // clear background
            QColor onColor(QColor::fromHsl(100, 160, 162));
            painter->setPen(Qt::NoPen);
            QLinearGradient onbg(0, 20, 0, 0);
            onbg.setSpread(QLinearGradient::ReflectSpread);
            onbg.setColorAt(0, onColor);
            onbg.setColorAt(1, onColor.darker(110));
            painter->setBrush(onbg);
            painter->drawRoundedRect(on, 5, 5);

            painter->setBrush(onColor.darker(120));
            painter->setPen(onColor.darker(110));
            painter->drawRect(on.center().x() - 7, (on.center().y() - (on.height()/2)) + 6, 2, on.height()-10);


            painter->setBrush(QBrush());
            painter->setPen(QPen(offColor.lighter(107), 3));
            painter->drawEllipse(QPoint(on.center().x() + on.width()-5, on.center().y()+1), (on.height()-12)/3, (on.height()-12)/2);


            float position = clamp(0, onOffOption->position, 1);
            QRect slider = on.translated((canvas.width() - on.width()) * position, 0);

            // draw frame
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::gray));
            painter->drawRoundedRect(slider, 5, 5);

            // clear background
            QColor sliderColour(248, 248, 248);
            painter->setPen(Qt::NoPen);
            QLinearGradient sliderbg(0, 20, 0, 0);
            sliderbg.setSpread(QLinearGradient::ReflectSpread);
            sliderbg.setColorAt(0, sliderColour);
            sliderbg.setColorAt(1, sliderColour.darker(105));
            painter->setBrush(sliderbg);
            painter->drawRoundedRect(slider, 5, 5);


            if(onOffOption->marked) {
                painter->setPen(QPen(sliderColour.darker(110)));
                painter->setBrush(QBrush(sliderColour.darker(115)));
                painter->drawRect(slider.right() - (slider.width()/2) -7, slider.y() + 5,
                                  1,  (slider.height() - 10));
                painter->drawRect(slider.right() - (slider.width()/2), slider.y() + 5,
                                  1,  (slider.height() - 10));
                painter->drawRect(slider.right() - (slider.width()/2) + 7, slider.y() + 5,
                                  1,  (slider.height() - 10));
            }
        } else {
            BASESTYLE::drawControl(control, option, painter, widget);
        }
    }
        return;

    case CE_ProgressBar: {

        const QStyleOptionProgressBarV2 * progressOptions = static_cast<const QStyleOptionProgressBarV2 *>(option);

        int hueRange = 100;
        int range = progressOptions->maximum - progressOptions->minimum;
        bool hasRange = (range != 0);
        if (range < 0) range = 0;

        float progress = 0;
        if(range != 0)
            progress = (float) (progressOptions->progress - progressOptions->minimum) / (float) range;

        int hue = hueRange * progress;
        if (progressOptions->invertedAppearance)
            hue = hueRange - hue;
        QColor progressColor(QColor::fromHsl(hue+1, 160, 162));

        painter->setRenderHint(QPainter::Antialiasing, true);
        QRect border = progressOptions->rect;
        border.setHeight(border.height()-4);
        border.setWidth(border.width()-4);
        border.setX(border.x()+4);
        border.setY(border.y()+4);

        QRect fill = border;
        fill.setWidth(fill.width()*progress);


        // draw frame
        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(Qt::gray));
        painter->drawRoundedRect(border, 5, 5);

        // clear background
        painter->setPen(Qt::NoPen);
        QLinearGradient bg(hasRange?20:0, 20, 0, 0);
        bg.setSpread(hasRange?QLinearGradient::RepeatSpread:QLinearGradient::ReflectSpread);
        bg.setColorAt(0, QColor(248, 248, 248));
        bg.setColorAt(1, QColor(248, 248, 248).darker(105));
        painter->setBrush(bg);
        painter->drawRoundedRect(border, 5, 5);


        // draw underlying fill frame
        //painter->setBrush(Qt::NoBrush);
        //painter->setPen(QPen(progressColor.darker(130)));
        //painter->drawRoundedRect(fill, 5, 5);

        // draw progress
        QLinearGradient gradient(0, 0, 10, 2);
        gradient.setSpread(QLinearGradient::ReflectSpread);
        gradient.setColorAt(0, progressColor);
        gradient.setColorAt(1, progressColor.darker(105));
        painter->setBrush(gradient);
        painter->drawRoundedRect(fill, 5, 5);


        if(progressOptions->textVisible){
            QFont font = painter->font();
            font.setBold(true);
            painter->setFont(font);
            painter->setPen(progressOptions->palette.color(QPalette::Text));
            int width = progressOptions->fontMetrics.width(progressOptions->text);
            int x = option->rect.x() + (option->rect.width()/2 - width/2);
            int height = progressOptions->fontMetrics.height();
            int y = option->rect.y() + (option->rect.height()/2 - height/2);
            QRect textRect(x, y, width, height);
            painter->drawText(textRect, progressOptions->text);
        }

        //BASESTYLE::drawControl(control, &outputOptions, painter, widget);
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
        const StyleOptionNeutralSpinBox *neutralSpin = qstyleoption_cast<const StyleOptionNeutralSpinBox *>(option);
        if(neutralSpin){
            QStyleOptionProgressBarV2 progressOptions;
            progressOptions.initFrom(widget);
            progressOptions.rect = option->rect;
            progressOptions.direction = option->direction;
            progressOptions.state = option->state;
            progressOptions.minimum = 0;
            progressOptions.maximum = 1000;
            progressOptions.progress = (int)(fabs(neutralSpin->level) * 1000.0);
            progressOptions.textVisible = false;
            progressOptions.invertedAppearance = neutralSpin->invertColours;

            drawControl(CE_ProgressBar, &progressOptions, painter, widget);
        }

        const StyleOptionGraphingSpinBox *graphingSpin = qstyleoption_cast<const StyleOptionGraphingSpinBox *>(option);
        if(graphingSpin){
            drawControl(CE_Graph, graphingSpin, painter, widget);
        }

        if(graphingSpin || neutralSpin){
            painter->setOpacity(0.5);
            painter->setBrush(QBrush(QColor(200,200,200)));
            int buttonSize = option->rect.height()-12;
            painter->drawRoundedRect(QRect(6, 6, buttonSize, buttonSize), 10, 10);
            painter->drawRoundedRect(QRect(option->rect.right()-(6+buttonSize), 6, buttonSize, buttonSize), 10, 10);

            painter->setBrush(QBrush(Qt::gray));
            painter->setPen(QPen(QBrush(Qt::gray), 3.0));
            painter->drawRoundRect(QRect(10, 5 + (buttonSize/2), buttonSize-9, 2), 4, 4);
            painter->drawRoundRect(QRect(option->rect.right()-(10+buttonSize) + 8, 5 + (buttonSize/2), buttonSize-8, 2), 3, 3);
            painter->drawRoundRect(QRect(option->rect.right()-(buttonSize/2) - 7, 10, 2, buttonSize-8), 3, 3);
            painter->setOpacity(1);
        }
    }
        break;
    default:
        BASESTYLE::drawComplexControl(control, option, painter, widget);
    }
}
