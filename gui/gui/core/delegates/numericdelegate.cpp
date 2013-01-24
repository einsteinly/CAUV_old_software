/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "numericdelegate.h"

#include <QtGui>

#include <algorithm>

#include "model/nodes/numericnode.h"

#include "widgets/neutralspinbox.h"
#include "styles/style.h"

using namespace cauv;
using namespace cauv::gui;


NumericDelegate::NumericDelegate(bool showTitle, QObject *parent) :
    AbstractNodeDelegate(parent),
    m_titles(showTitle) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);

    //https://bugreports.qt-project.org/browse/QTBUG-2151
    qRegisterMetaType<cauv::BoundedFloat>("cauv__BoundedFloat");

    factory->registerEditor(QVariant::Int,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::UInt,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::Double,
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(),
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<cauv::BoundedFloat>(),
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
    //AbstractNodeDelegate::setEditorData(editor, index);

    NumericNodeBase * node = static_cast<NumericNodeBase*>(index.internalPointer());

    if(!m_updatesWhileEditing) {
        if(editor->property("data-initialised").toBool()) return;
        editor->setProperty("data-initialised", true);
    }

    if(node) {
        if(NeutralSpinBox * neutral = qobject_cast<NeutralSpinBox*>(editor)){
            if(node->isMaxSet() && node->isMinSet()){
                neutral->setMinimum(node->getMin().toInt());
                neutral->setMaximum(node->getMax().toInt());
                neutral->setWrapping(node->getWraps());
                neutral->setNeutral(node->getNeutral().toInt());
                neutral->setInverted(node->isInverted());
            }
            // hack
            neutral->setValue(node->asNumber().toInt());
        }


        if(NeutralDoubleSpinBox * neutral = qobject_cast<NeutralDoubleSpinBox*>(editor)){
            if(node->isMaxSet() && node->isMinSet()){
                neutral->setMinimum(node->getMin().toDouble());
                neutral->setMaximum(node->getMax().toDouble());
                neutral->setWrapping(node->getWraps());
                neutral->setNeutral(node->getNeutral().toDouble());
            }
            neutral->setDecimals(node->getPrecision());
            neutral->setInverted(node->isInverted());

            // hack
            neutral->setValue(node->asNumber().toDouble());
        }
    }
}



QSize NumericDelegate::sizeHint(const QStyleOptionViewItem & option,
                       const QModelIndex &index) const{
    return QSize(std::max(titleRect(option, index).width() * 1.2, 140.0),
                 option.fontMetrics.height()*1.5);
}

QRect NumericDelegate::controlRect(const QStyleOptionViewItem &option,
                          const QModelIndex &index) const {
    if(m_titles)
        return option.rect;

    return AbstractNodeDelegate::controlRect(option, index);
}
