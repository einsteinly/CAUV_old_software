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

#ifndef CAUVGUI_H
#define CAUVGUI_H

#include <QMainWindow>
#include <boost/enable_shared_from_this.hpp>
#include <common/cauv_node.h>

#include <QtGui>

#include "guiactions.h"

#define BASESTYLE QPlastiqueStyle

namespace Ui {
class MainWindow;
}
class QDir;

namespace cauv {
namespace gui {

class CauvInterfacePlugin;






class CauvStyle : public BASESTYLE
{
    Q_OBJECT

public:
    CauvStyle() {}

    void polish(QPalette &palette){
        /*QColor brown(212, 140, 95);
                QColor beige(236, 182, 120);
                QColor slightlyOpaqueBlack(0, 0, 0, 63);

                QPixmap backgroundImage(":/images/woodbackground.png");
                QPixmap buttonImage(":/images/woodbutton.png");
                QPixmap midImage = buttonImage;

                QPainter painter;
                painter.begin(&midImage);
                painter.setPen(Qt::NoPen);
                painter.fillRect(midImage.rect(), slightlyOpaqueBlack);
                painter.end();

                palette = QPalette(brown);

                palette.setBrush(QPalette::BrightText, Qt::white);
                palette.setBrush(QPalette::Base, beige);
                palette.setBrush(QPalette::Highlight, Qt::darkGreen);
                setTexture(palette, QPalette::Button, buttonImage);
                setTexture(palette, QPalette::Mid, midImage);
                setTexture(palette, QPalette::Window, backgroundImage);

                QBrush brush = palette.background();
                brush.setColor(brush.color().dark());

                palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush);
                palette.setBrush(QPalette::Disabled, QPalette::Text, brush);
                palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush);
                palette.setBrush(QPalette::Disabled, QPalette::Base, brush);
                palette.setBrush(QPalette::Disabled, QPalette::Button, brush);
                palette.setBrush(QPalette::Disabled, QPalette::Mid, brush);*/

        BASESTYLE::polish(palette);
    }

    void polish(QWidget *widget){
        //if (qobject_cast<QPushButton *>(widget)
        //        || qobject_cast<QComboBox *>(widget))
        //    widget->setAttribute(Qt::WA_Hover, true);

        BASESTYLE::polish(widget);
    }

    void unpolish(QWidget *widget){
        //if (qobject_cast<QPushButton *>(widget)
        //         || qobject_cast<QComboBox *>(widget))
        //     widget->setAttribute(Qt::WA_Hover, false);

        BASESTYLE::unpolish(widget);
    }


    int pixelMetric(PixelMetric metric, const QStyleOption *option,
                    const QWidget *widget) const{
        return BASESTYLE::pixelMetric(metric, option, widget);
    }

    int styleHint(StyleHint hint, const QStyleOption *option,
                  const QWidget *widget, QStyleHintReturn *returnData) const{
        //switch (hint) {
        //case SH_DitherDisabledText:
        //    return int(false);
        //case SH_EtchDisabledText:
        //    return int(true);
        //default:
        return BASESTYLE::styleHint(hint, option, widget, returnData);
        //}
    }

    void drawPrimitive(PrimitiveElement element, const QStyleOption *option,
                       QPainter *painter, const QWidget *widget) const{
        /*switch (element) {
                     case PE_PanelButtonCommand:
                         {
                             int delta = (option->state & State_MouseOver) ? 64 : 0;
                             QColor slightlyOpaqueBlack(0, 0, 0, 63);
                             QColor semiTransparentWhite(255, 255, 255, 127 + delta);
                             QColor semiTransparentBlack(0, 0, 0, 127 - delta);

                             int x, y, width, height;
                             option->rect.getRect(&x, &y, &width, &height);

                             QPainterPath roundRect = roundRectPath(option->rect);
                             int radius = qMin(width, height) / 2;

                             QBrush brush;
                             bool darker;

                             const QStyleOptionButton *buttonOption =
                                     qstyleoption_cast<const QStyleOptionButton *>(option);
                             if (buttonOption
                                     && (buttonOption->features & QStyleOptionButton::Flat)) {
                                 brush = option->palette.background();
                                 darker = (option->state & (State_Sunken | State_On));
                             } else {
                                 if (option->state & (State_Sunken | State_On)) {
                                     brush = option->palette.mid();
                                     darker = !(option->state & State_Sunken);
                                 } else {
                                     brush = option->palette.button();
                                     darker = false;
                                 }
                             }

                             painter->save();
                             painter->setRenderHint(QPainter::Antialiasing, true);
                             painter->fillPath(roundRect, brush);
                             if (darker)
                                 painter->fillPath(roundRect, slightlyOpaqueBlack);

                             int penWidth;
                             if (radius < 10)
                                 penWidth = 3;
                             else if (radius < 20)
                                 penWidth = 5;
                             else
                                 penWidth = 7;

                             QPen topPen(semiTransparentWhite, penWidth);
                             QPen bottomPen(semiTransparentBlack, penWidth);

                             if (option->state & (State_Sunken | State_On))
                                 qSwap(topPen, bottomPen);

                             int x1 = x;
                             int x2 = x + radius;
                             int x3 = x + width - radius;
                             int x4 = x + width;

                             if (option->direction == Qt::RightToLeft) {
                                 qSwap(x1, x4);
                                 qSwap(x2, x3);
                             }

                             QPolygon topHalf;
                             topHalf << QPoint(x1, y)
                                     << QPoint(x4, y)
                                     << QPoint(x3, y + radius)
                                     << QPoint(x2, y + height - radius)
                                     << QPoint(x1, y + height);

                             painter->setClipPath(roundRect);
                             painter->setClipRegion(topHalf, Qt::IntersectClip);
                             painter->setPen(topPen);
                             painter->drawPath(roundRect);

                             QPolygon bottomHalf = topHalf;
                             bottomHalf[0] = QPoint(x4, y + height);

                             painter->setClipPath(roundRect);
                             painter->setClipRegion(bottomHalf, Qt::IntersectClip);
                             painter->setPen(bottomPen);
                             painter->drawPath(roundRect);

                             painter->setPen(option->palette.foreground().color());
                             painter->setClipping(false);
                             painter->drawPath(roundRect);

                             painter->restore();
                         }
                         break;
                     default:*/
        BASESTYLE::drawPrimitive(element, option, painter, widget);
        //}
    }

    QRect subControlRect ( ComplexControl control, const QStyleOptionComplex * option, SubControl subControl, const QWidget * widget = 0 ) const {
        switch(subControl) {
        case QStyle::SC_SpinBoxUp: {
            QRect frame = BASESTYLE::subControlRect(control, option, SC_SpinBoxFrame, widget);
            QRect rect = BASESTYLE::subControlRect(control, option, subControl, widget);
            //rect.setY(frame.y());
            rect.setX((frame.x()+frame.width())-frame.height());
            rect.setWidth(frame.height());
            rect.setHeight(frame.height());

            //rect.setHeight(frame.height());
            //rect.setWidth(frame.height());
            //rect.setRight(frame.left()-rect.height());
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

            //rect.setWidth(frame.height());
            //rect.setHeight(frame.height());
            //rect.setY(frame.y());
            //rect.setX(frame.x());
            return rect;
        }
        break;

        case QStyle::SC_SpinBoxEditField: {
            QRect frame = BASESTYLE::subControlRect(control, option, SC_SpinBoxFrame, widget);
            QRect rect = BASESTYLE::subControlRect(control, option, subControl, widget);
            rect.setLeft(rect.height()+10);
            rect.setRight(frame.right()-frame.height()-10);
            return rect;
        }
        break;

        default: return BASESTYLE::subControlRect(control, option, subControl, widget);
        }
    }

    void drawControl(ControlElement control, const QStyleOption *option,
                     QPainter *painter, const QWidget *widget) const{

        /*switch (control) {
                    case CE_PushButtonLabel:
                        {
                            QStyleOptionButton myButtonOption;
                            const QStyleOptionButton *buttonOption =
                                    qstyleoption_cast<const QStyleOptionButton *>(option);
                            if (buttonOption) {
                                myButtonOption = *buttonOption;
                                if (myButtonOption.palette.currentColorGroup()
                                        != QPalette::Disabled) {
                                    if (myButtonOption.state & (State_Sunken | State_On)) {
                                        myButtonOption.palette.setBrush(QPalette::ButtonText,
                                                myButtonOption.palette.brightText());
                                    }
                                }
                            }
                            BASESTYLE::drawControl(control, &myButtonOption, painter, widget);
                        }
                        break;
                    default:*/
        BASESTYLE::drawControl(control, option, painter, widget);
        //}
    }

private:
    static void setTexture(QPalette &palette, QPalette::ColorRole role,
                           const QPixmap &pixmap){
        for (int i = 0; i < QPalette::NColorGroups; ++i) {
            QColor color = palette.brush(QPalette::ColorGroup(i), role).color();
            palette.setBrush(QPalette::ColorGroup(i), role, QBrush(color, pixmap));
        }
    }

    static QPainterPath roundRectPath(const QRect &rect){
        int radius = qMin(rect.width(), rect.height()) / 2;
        int diam = 2 * radius;

        int x1, y1, x2, y2;
        rect.getCoords(&x1, &y1, &x2, &y2);

        QPainterPath path;
        path.moveTo(x2, y1 + radius);
        path.arcTo(QRect(x2 - diam, y1, diam, diam), 0.0, +90.0);
        path.lineTo(x1 + radius, y1);
        path.arcTo(QRect(x1, y1, diam, diam), 90.0, +90.0);
        path.lineTo(x1, y2 - radius);
        path.arcTo(QRect(x1, y2 - diam, diam, diam), 180.0, +90.0);
        path.lineTo(x1 + radius, y2);
        path.arcTo(QRect(x2 - diam, y2 - diam, diam, diam), 270.0, +90.0);
        path.closeSubpath();
        return path;
    }
};














class CauvMainWindow : public QMainWindow, public CauvNode, public boost::enable_shared_from_this<CauvMainWindow> {
    Q_OBJECT

public:
    CauvMainWindow(QApplication * app);
    virtual ~CauvMainWindow();

public Q_SLOTS:
    int send(boost::shared_ptr<const Message>message);

protected:
    virtual void onRun();
    virtual CauvInterfacePlugin * loadPlugin(QObject * plugin);
    virtual void closeEvent(QCloseEvent *);

    QApplication * m_application;
    // actions are the way the GUI exposes itself and the vehicles model to plugins
    boost::shared_ptr<GuiActions> m_actions;
    std::vector<CauvInterfacePlugin *> m_plugins;

private:
    Ui::MainWindow * ui;
    int findPlugins(const QDir& dir, int subdirs = 0);

};

} //namespace gui
} // namespace cauv

#endif // CAUVGUI_H
