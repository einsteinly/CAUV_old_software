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

#include "delegates.h"

#include <QItemEditorCreator>
#include <QApplication>

#include "model/nodes/numericnode.h"
#include "widgets/neutralspinbox.h"
#include "widgets/graphbar.h"
#include "model/utils/sampler.h"
#include "style.h"

using namespace cauv;
using namespace cauv::gui;


NodeDelegateMapper::NodeDelegateMapper(QObject *parent){}

void NodeDelegateMapper::registerDelegate(node_type nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate){
    m_default_delegates[nodeType] = delegate;
}

void NodeDelegateMapper::registerDelegate(boost::shared_ptr<Node> node, boost::shared_ptr<QAbstractItemDelegate> delegate){
    m_delegates[node] = delegate;
}

boost::shared_ptr<QAbstractItemDelegate> NodeDelegateMapper::getDelegate(boost::shared_ptr<Node> node) const {
    try {
        // specific delegate first
        return m_delegates.at(node);
    } catch (std::out_of_range){
        // general delegate next
        return m_default_delegates.at(node->type);
    }
}

QWidget * NodeDelegateMapper::createEditor ( QWidget * parent, const QStyleOptionViewItem & option,
                                             const QModelIndex & index ) const {
    const boost::shared_ptr<Node> node = static_cast<Node*>(index.internalPointer())->shared_from_this();
    try {
        boost::shared_ptr<QAbstractItemDelegate> delegate = getDelegate(node);
        return delegate->createEditor(parent, option, index);
    } catch (std::out_of_range){
        return QStyledItemDelegate::createEditor(parent,option, index);
    }
}

void NodeDelegateMapper::paint(QPainter *painter, const QStyleOptionViewItem &option,
                               const QModelIndex &index) const{

    // display the node
    const boost::shared_ptr<Node> node = static_cast<Node*>(index.internalPointer())->shared_from_this();
    if (node && index.column() == 1) {
        try {
            boost::shared_ptr<QAbstractItemDelegate> delegate = getDelegate(node);
            delegate->paint(painter, option, index);
        } catch (std::out_of_range){
            QStyledItemDelegate::paint(painter, option, index);
        }
    } else QStyledItemDelegate::paint(painter, option, index);
}

QSize NodeDelegateMapper::sizeHint(const QStyleOptionViewItem &option,
                                   const QModelIndex &index) const{
    return QSize(100, 30);
}




QMap<QModelIndex, boost::shared_ptr<SampleQueue<QVariant> > > GraphingDelegate::m_samplers;

GraphingDelegate::GraphingDelegate(QObject * parent) : QStyledItemDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    this->setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Int, new QItemEditorCreator<GraphingSpinBox>("value"));
    factory->registerEditor(QVariant::UInt, new QItemEditorCreator<GraphingSpinBox>("value"));
    factory->registerEditor(QVariant::Double, new QItemEditorCreator<GraphingDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(), new QItemEditorCreator<GraphingDoubleSpinBox>("value"));
}

void GraphingDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                             const QModelIndex &index) const
{
    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());
    if (node && index.column() == 1 && node->isMaxSet() && node->isMinSet()) {

        if(!m_samplers.contains(index)){
            m_samplers[index] = boost::make_shared<SampleQueue<QVariant> >(boost::bind(&Node::get, node));
        }

        if(!m_samplers.contains(index)){
            error() << "GraphingDelegate: Can't find sampler!";
            return;
        }
        StyleOptionGraphingWidget graphingOption;
        graphingOption.rect = option.rect;
        graphingOption.maximum = node->getMax();
        graphingOption.minimum = node->getMin();
        graphingOption.samples = m_samplers[index]->samples();
        QApplication::style()->drawControl((QStyle::ControlElement)CauvStyle::CE_Graph, &graphingOption, painter);

    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }

}

QWidget * GraphingDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                         const QModelIndex &index) const{
    QWidget * editor = QStyledItemDelegate::createEditor(parent, option, index);

    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());

    if(node && node->isMaxSet() && node->isMinSet()) {

        if(GraphingSpinBox * neutral = qobject_cast<GraphingSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toInt());
            neutral->setMaximum(node->getMax().toInt());
            neutral->setWrapping(node->getWraps());
            neutral->setSampler(m_samplers[index]);
        }

        if(GraphingDoubleSpinBox * neutral = qobject_cast<GraphingDoubleSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toDouble());
            neutral->setMaximum(node->getMax().toDouble());
            neutral->setWrapping(node->getWraps());
            neutral->setDecimals(node->getPrecision());
            neutral->setSampler(m_samplers[index]);
        }
    }

    return editor;
}




ProgressDelegate::ProgressDelegate(QObject * parent) : QStyledItemDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    this->setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Int, new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::UInt, new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::Double, new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(), new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
}

void ProgressDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                             const QModelIndex &index) const
{
    const NumericNodeBase * node = dynamic_cast<const NumericNodeBase*>((Node*)index.internalPointer());
    if (node && index.column() == 1 && node->isMaxSet() && node->isMinSet()) {
        QStyleOptionProgressBarV2 progressBarOption;
        progressBarOption.rect = option.rect;
        progressBarOption.text = QString::number(index.data().value<double>()).append(QString::fromStdString(node->getUnits()));
        progressBarOption.textVisible = true;
        progressBarOption.invertedAppearance = true;
        progressBarOption.maximum = 1000;
        progressBarOption.minimum = 0;
        progressBarOption.progress = (int)(fabs(pivot(node->getMin().toDouble(),
                                                      node->getNeutral().toDouble(),
                                                      node->getMax().toDouble(),
                                                      node->get().toDouble()))*progressBarOption.maximum);

        QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                           &progressBarOption, painter);
    } else
        QStyledItemDelegate::paint(painter, option, index);

}

QWidget * ProgressDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                         const QModelIndex &index) const{
    QWidget * editor = QStyledItemDelegate::createEditor(parent, option, index);

    const NumericNodeBase * node = dynamic_cast<const NumericNodeBase*>((Node*)index.internalPointer());

    if(node && node->isMaxSet() && node->isMinSet()) {
        if(NeutralSpinBox * neutral = qobject_cast<NeutralSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toInt());
            neutral->setMaximum(node->getMax().toInt());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toInt());
        }

        if(NeutralDoubleSpinBox * neutral = qobject_cast<NeutralDoubleSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toDouble());
            neutral->setMaximum(node->getMax().toDouble());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toDouble());
            neutral->setDecimals(node->getPrecision());
        }
    }

    return editor;
}





HybridDelegate::HybridDelegate(QObject * parent) : GraphingDelegate(parent), ProgressDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Int, new QItemEditorCreator<GraphingSpinBox>("value"));
    factory->registerEditor(QVariant::UInt, new QItemEditorCreator<GraphingSpinBox>("value"));
    factory->registerEditor(QVariant::Double, new QItemEditorCreator<GraphingDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(), new QItemEditorCreator<GraphingDoubleSpinBox>("value"));
}


void HybridDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                             const QModelIndex &index) const
{

    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());
    if (node && index.column() == 1 && node->isMaxSet() && node->isMinSet()) {
        if(!m_samplers.contains(index)){
            m_samplers[index] = boost::make_shared<SampleQueue<QVariant> >(boost::bind(&Node::get, node));
        }
    }

    ProgressDelegate::paint(painter, option, index);
}

QWidget * HybridDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                                       const QModelIndex &index) const{
    return GraphingDelegate::createEditor(parent, option, index);
}
