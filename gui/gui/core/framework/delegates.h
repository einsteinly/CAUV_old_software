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

#ifndef DELEGATES_H
#define DELEGATES_H

#include <style.h>

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/widgets/neutralspinbox.h>
#include <gui/core/model/model.h>

#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QApplication>

#include <common/bounded_float.h>

namespace cauv {
    namespace gui {


    class NodeDelegateMapper : public QStyledItemDelegate
    {
        Q_OBJECT

    public:
        NodeDelegateMapper(QObject *parent = 0){
        }

        void registerDelegate(node_type nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate){
            m_default_delegates[nodeType] = delegate;
        }

        void registerDelegate(boost::shared_ptr<Node> node, boost::shared_ptr<QAbstractItemDelegate> delegate){
            m_delegates[node] = delegate;
        }

        boost::shared_ptr<QAbstractItemDelegate> getDelegate(boost::shared_ptr<Node> node) const {
            try {
                // specific delegate first
                return m_delegates.at(node);
            } catch (std::out_of_range){
                // general delegate next
                return m_default_delegates.at(node->type);
            }
        }

        QWidget * createEditor ( QWidget * parent, const QStyleOptionViewItem & option,
                                 const QModelIndex & index ) const {
            const boost::shared_ptr<Node> node = static_cast<Node*>(index.internalPointer())->shared_from_this();
            try {
                boost::shared_ptr<QAbstractItemDelegate> delegate = getDelegate(node);
                return delegate->createEditor(parent, option, index);
            } catch (std::out_of_range){
                return QStyledItemDelegate::createEditor(parent,option, index);
            }
        }


        void paint(QPainter *painter, const QStyleOptionViewItem &option,
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

        QSize sizeHint(const QStyleOptionViewItem &option,
                                     const QModelIndex &index) const{
            return QSize(100, 30);
        }

    protected:
        std::map<boost::shared_ptr<Node>, boost::shared_ptr<QAbstractItemDelegate> > m_delegates;
        std::map<node_type, boost::shared_ptr<QAbstractItemDelegate> > m_default_delegates;
    };



    struct ProgressBarDelegate : public QStyledItemDelegate {

        ProgressBarDelegate(QObject * parent = 0) : QStyledItemDelegate(parent) {
            QItemEditorFactory * factory = new QItemEditorFactory();
            this->setItemEditorFactory(factory);
            factory->registerEditor(QVariant::Int, new QItemEditorCreator<NeutralSpinBox>("value"));
            factory->registerEditor(QVariant::UInt, new QItemEditorCreator<NeutralSpinBox>("value"));
            factory->registerEditor(QVariant::Double, new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
            factory->registerEditor((QVariant::Type)qMetaTypeId<float>(), new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
        }

        void paint(QPainter *painter, const QStyleOptionViewItem &option,
                                   const QModelIndex &index) const
        {
            const NumericNodeBase * node = dynamic_cast<const NumericNodeBase*>((Node*)index.internalPointer());
            if (node && index.column() == 1 && node->isMaxSet() && node->isMinSet()) {
                QStyleOptionProgressBarV2 progressBarOption;
                progressBarOption.rect = option.rect;
                progressBarOption.progress = index.data().toInt();
                progressBarOption.text = QString::number(index.data().value<double>()).append(QString::fromStdString(node->getUnits()));
                progressBarOption.textVisible = true;
                info() << "max" << (progressBarOption.maximum = node->getMax().toInt());
                info() << "min" << (progressBarOption.minimum = node->getMin().toInt());

                QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                                   &progressBarOption, painter);
            } else
                QStyledItemDelegate::paint(painter, option, index);

        }

        QWidget * createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const{
            QWidget * editor = QStyledItemDelegate::createEditor(parent, option, index);

            const NumericNodeBase * node = dynamic_cast<const NumericNodeBase*>((Node*)index.internalPointer());

            debug() << "Created editor: " << editor->metaObject()->className();

            if(node && node->isMaxSet() && node->isMinSet()) {
                if(NeutralSpinBox * neutral = qobject_cast<NeutralSpinBox*>(editor)){
                    neutral->setMinimum(node->getMin().toInt());
                    neutral->setMaximum(node->getMax().toInt());
                    neutral->setWrapping(node->getWraps());
                    neutral->setNeutral(node->getMin().toInt()); //!!! todo
                }

                if(NeutralDoubleSpinBox * neutral = qobject_cast<NeutralDoubleSpinBox*>(editor)){
                    neutral->setMinimum(node->getMin().toDouble());
                    neutral->setMaximum(node->getMax().toDouble());
                    neutral->setWrapping(node->getWraps());
                    neutral->setNeutral(node->getMin().toFloat()); //!!! todo
                }
            }

            return editor;
        }
    };





    } // namespace gui
} // namespace cauv


#endif // CAUV_STYLE_H
