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

#include "colourdelegate.h"

#include <QtGui>

#include <algorithm>

#include "model/nodes/colournode.h"

using namespace cauv;
using namespace cauv::gui;


ColourDelegate::ColourDelegate(QObject * parent) :
    AbstractNodeDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor((QVariant::Type)qMetaTypeId<Colour>(),
                            new QItemEditorCreator<ColourDialog>("colour"));
}


void ColourDelegate::paint(QPainter *painter,
                           const QStyleOptionViewItem &option,
                           const QModelIndex &index) const
{
    AbstractNodeDelegate::paint(painter, option, index);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing);
    Colour colour = index.model()->data(index, Qt::EditRole).value<Colour>();
    painter->setBrush(QColor::fromRgbF(colour.r(), colour.g(), colour.b(), colour.a()));
    painter->setPen(QPen(QColor(150,150,150), 2));
    QRect rect = controlRect(option,index).adjusted(0,2,0,-2);
    painter->drawEllipse(QRect(rect.x(), rect.y(),
                               rect.height()-2, rect.height()-2));
    painter->setPen(Qt::black);
    QString text("%1 %2 %3");
    text = text.arg(QString::number(colour.r(), 'f', 2)).
            arg(QString::number(colour.g(), 'f', 2)).
            arg(QString::number(colour.b(), 'f', 2));
    rect = rect.adjusted(rect.height()+2,0,0,0);
    painter->drawText(rect, text, QTextOption(Qt::AlignVCenter));
    painter->restore();
}

void ColourDelegate::setEditorData(QWidget *editor,
                                   const QModelIndex &index) const{
    // don't allow updates while editing
    if(editor->property("data-initialised").toBool()) return;
    editor->setProperty("data-initialised", true);

    QStyledItemDelegate::setEditorData(editor, index);

    Colour colour = index.model()->data(index, Qt::EditRole).value<Colour>();

    ColourDialog * dialog = static_cast<ColourDialog *>(editor);
    dialog->setColour(colour);
}



QSize ColourDelegate::sizeHint(const QStyleOptionViewItem &option,
                                       const QModelIndex &) const{
    return QSize(120, option.fontMetrics.height()*2);
}
