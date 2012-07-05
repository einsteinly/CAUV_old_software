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

#include "numericdelegate.h"

#include <QtGui>

#include <algorithm>

#include "model/nodes/numericnode.h"

#include "widgets/neutralspinbox.h"
#include "framework/style.h"

using namespace cauv;
using namespace cauv::gui;

NumericDelegate::NumericDelegate(bool showTitle, QObject *parent) :
    AbstractNodeDelegate(parent),
    m_titles(showTitle) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Int,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::UInt,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::Double,
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(),
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<BoundedFloat>(),
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
}

void NumericDelegate::paint(QPainter *painter,
                            const QStyleOptionViewItem &option,
                            const QModelIndex &index) const
{
    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());
    if (node) {
        QString value = QString::number( node->asNumber().toDouble(),
                                         'g', node->getPrecision() );
        value.append(QString::fromStdString(node->getUnits()));

        QString text;

        if(m_titles) {
            text = QString::fromStdString(node->nodeName()).append(":").append(value);
        } else {
            text = value;
        }

        QStyleOptionProgressBarV2 progressBarOption;
        progressBarOption.rect = controlRect(option, index);
        progressBarOption.text = text;
        progressBarOption.textVisible = true;
        progressBarOption.invertedAppearance = node->isInverted();
        progressBarOption.palette.setColor(QPalette::Text, Qt::black);
        if(!node->isMutable())
            progressBarOption.palette.setColor(QPalette::Text, QColor(100,100,100));

        progressBarOption.maximum = (node->isMaxSet() && node->isMinSet()) ? 1000 : 0;
        progressBarOption.minimum = 0;
        progressBarOption.progress = (int)(fabs(pivot(node->getMin().toDouble(),
                                                      node->getNeutral().toDouble(),
                                                      node->getMax().toDouble(),
                                                      node->asNumber().toDouble()))*progressBarOption.maximum);

        QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                           &progressBarOption, painter);
    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }
}

void NumericDelegate::commit() {
    emit commitData(static_cast<QWidget *>(sender()));
}

void NumericDelegate::setEditorData(QWidget *editor,
                                    const QModelIndex &index) const{
    // don't allow updates while editing
    if(editor->property("data-initialised").toBool()) return;
    editor->setProperty("data-initialised", true);

    QStyledItemDelegate::setEditorData(editor, index);

    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());

    if(node && node->isMaxSet() && node->isMinSet()) {
        if(NeutralSpinBox * neutral = qobject_cast<NeutralSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toInt());
            neutral->setMaximum(node->getMax().toInt());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toInt());
            neutral->setInverted(node->isInverted());
        }

        if(NeutralDoubleSpinBox * neutral = qobject_cast<NeutralDoubleSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toDouble());
            neutral->setMaximum(node->getMax().toDouble());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toDouble());
            neutral->setDecimals(node->getPrecision());
            neutral->setInverted(node->isInverted());
        }
    }
}



QSize NumericDelegate::sizeHint(const QStyleOptionViewItem & option,
                       const QModelIndex &index) const{
    return QSize(titleRect(option, index).width() * 1.2, option.fontMetrics.height()*1.5);
}

QRect NumericDelegate::controlRect(const QStyleOptionViewItem &option,
                          const QModelIndex &index) const {
    if(m_titles)
        return option.rect;

    return AbstractNodeDelegate::controlRect(option, index);
}
