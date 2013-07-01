/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "colourdelegate.h"

#include <QtGui>

#include <algorithm>

#include "model/nodes/colournode.h"

using namespace cauv;
using namespace cauv::gui;


ColourDelegate::ColourDelegate(QObject * parent) :
    AbstractNodeDelegate(parent) {
}

QWidget * ColourDelegate::createEditor(QWidget *parent,
                                       const QStyleOptionViewItem &option,
                                       const QModelIndex &index) const {
    Q_UNUSED(parent);
    Q_UNUSED(option);

    Colour colour = index.model()->data(index, Qt::EditRole).value<Colour>();
    auto  colourDialog =  new ColourDialog();
    colourDialog->setColour(colour);
    return colourDialog;
}

void ColourDelegate::paint(QPainter *painter,
                           const QStyleOptionViewItem &option,
                           const QModelIndex &index) const
{
    AbstractNodeDelegate::paint(painter, option, index);

    //todo: clean this up!

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing);
    Colour colour = index.model()->data(index, Qt::EditRole).value<Colour>();
    painter->setBrush(QColor::fromRgbF(colour.r(), colour.g(), colour.b(), colour.a()));
    painter->setPen(QPen(QColor(150,150,150), 2));
    QRect rect = controlRect(option,index).adjusted(3,2,0,-2);
    qreal height = painter->fontMetrics().height()*1.3;
    painter->drawEllipse(QRect(rect.x(), rect.y() + rect.height()/2 - height/2,
                               height, height));
    painter->setPen(Qt::black);
    QFont font = painter->font();
    int channels = 3;
    if (colour.type == ColourType::Grey)
        channels = 1;
    int margin = rect.height()*((channels == 1) ? 0.3 : 0.1);
    int lineHeight = (rect.height()-margin-margin)/channels;
    font.setPixelSize(lineHeight);
    rect.setHeight(lineHeight);
    rect = rect.adjusted(0, margin, 0, margin);
    painter->setFont(font);
    rect = rect.adjusted(height+4,0,0,0);

    for(int i = 0; i < channels; i ++){

        QString text("%1");
        switch(i){
        case 0:
            painter->setPen(Qt::black);
            text = text.arg(QString::number(colour.a(), 'f', 2));
        break;
        case 1:
            painter->setPen(Qt::red);
            text = text.arg(QString::number(colour.r(), 'f', 2));
        break;
        case 2:
            painter->setPen(Qt::green);
            text = text.arg(QString::number(colour.g(), 'f', 2));
        break;
        default:
            painter->setPen(Qt::blue);
            text = text.arg(QString::number(colour.b(), 'f', 2));
        break;
        }

        QTextOption to(Qt::AlignVCenter);
        to.setWrapMode(QTextOption::NoWrap);
        painter->drawText(rect, text, to);

        rect = rect.adjusted(0,lineHeight,0,lineHeight);
    }
    painter->restore();
}

void ColourDelegate::setEditorData(QWidget *editor,
                                   const QModelIndex &index) const{
    QStyledItemDelegate::setEditorData(editor, index);

    ColourDialog * dialog = static_cast<ColourDialog *>(editor);
    Colour oldColour = dialog->colour();

    Colour colour = index.model()->data(index, Qt::EditRole).value<Colour>();
    colour.type = oldColour.type;

    dialog->setColour(colour);
}



QSize ColourDelegate::sizeHint(const QStyleOptionViewItem & option,
                               const QModelIndex &) const{
    return QSize(120, option.fontMetrics.height()*2);
}
