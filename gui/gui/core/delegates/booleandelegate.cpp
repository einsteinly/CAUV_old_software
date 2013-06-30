/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "booleandelegate.h"

#include <QtGui>

#include "model/nodes/numericnode.h"

#include "widgets/onoff.h"
#include "styles/style.h"

using namespace cauv;
using namespace cauv::gui;


BooleanDelegate::BooleanDelegate(QObject * parent) :
    AbstractNodeDelegate(parent) {
    auto factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Bool, new QItemEditorCreator<OnOffSlider>("checked"));
    m_updatesWhileEditing = true;
}


void BooleanDelegate::paint(QPainter *painter,
                            const QStyleOptionViewItem &option,
                            const QModelIndex &index) const
{
    QRect title = titleRect(option, index);
    int width = controlRect(option, index).width();
    title = title.adjusted(width+2, 0, width+2, 0);
    QStyleOptionViewItem modifiedOptions = option;
    modifiedOptions.rect = title;

    AbstractNodeDelegate::paint(painter, modifiedOptions, index);

    BooleanNode * node = dynamic_cast<BooleanNode*>((Node*)index.internalPointer());
    if (node) {
        StyleOptionOnOff onOffOption;
        onOffOption.rect = controlRect(option, index);
        onOffOption.position = node->typedGet();
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
        node->set(!node->typedGet());
    }
    return retval;
}

void BooleanDelegate::commit() {
    emit commitData(static_cast<QWidget *>(sender()));
}

QSize BooleanDelegate::sizeHint(const QStyleOptionViewItem & option,
                                const QModelIndex &index) const{
    return QSize(titleRect(option, index).size().width() + controlRect(option,index).width(),
                 controlRect(option,index).height());
}


QRect BooleanDelegate::controlRect(const QStyleOptionViewItem & option,
                  const QModelIndex &) const{
    QRect rect = option.rect;
    return QRect(rect.x() + 10, rect.y(), 65, 25);
}
