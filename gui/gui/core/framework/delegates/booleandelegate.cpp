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

#include "booleandelegate.h"

#include <QtGui>

#include "model/nodes/numericnode.h"

#include "widgets/onoff.h"
#include "framework/style.h"

using namespace cauv;
using namespace cauv::gui;


BooleanDelegate::BooleanDelegate(QObject * parent) :
    AbstractNodeDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Bool, new QItemEditorCreator<OnOffSlider>("checked"));
}


void BooleanDelegate::paint(QPainter *painter,
                            const QStyleOptionViewItem &option,
                            const QModelIndex &index) const
{
    AbstractNodeDelegate::paint(painter, option, index);

    BooleanNode * node = dynamic_cast<BooleanNode*>((Node*)index.internalPointer());
    if (node) {
        StyleOptionOnOff onOffOption;
        onOffOption.rect = controlRect(option, index);
        onOffOption.position = node->get();
        onOffOption.marked = node->isMutable();
        QApplication::style()->drawControl(QStyle::CE_CheckBox,
                                           &onOffOption, painter);
    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }
}


QWidget * BooleanDelegate::createEditor(QWidget *parent,
                                        const QStyleOptionViewItem &option,
                                        const QModelIndex &index) const{
    QWidget *retval = QStyledItemDelegate::createEditor(parent, option, index);
    if (OnOffSlider * slider = dynamic_cast<OnOffSlider*>(retval)) {
        slider->setAnimation(true);
        connect(retval, SIGNAL(switched()), this, SLOT(commit()));
        BooleanNode * node = dynamic_cast<BooleanNode*>((Node*)index.internalPointer());
        node->set(!node->get());
    }
    return retval;
}

void BooleanDelegate::commit() {
    emit commitData(static_cast<QWidget *>(sender()));
}

QSize BooleanDelegate::sizeHint(const QStyleOptionViewItem & option,
                                const QModelIndex &index) const{
    return QSize(titleRect(option, index).size().width() + 65 , 25);
}


QRect BooleanDelegate::controlRect(const QStyleOptionViewItem &option,
                  const QModelIndex &index) const{
    QRect rect = AbstractNodeDelegate::controlRect(option, index);
    rect.setSize(QSize(65, 25));
    return rect;
}
