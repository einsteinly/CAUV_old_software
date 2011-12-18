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


        void registerDelegate(GuiNodeType::e nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate){
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
        std::map<GuiNodeType::e, boost::shared_ptr<QAbstractItemDelegate> > m_default_delegates;
    };



    struct ProgressBarDelegate : public QStyledItemDelegate {

        ProgressBarDelegate(QObject * parent = 0) : QStyledItemDelegate(parent) {
            QItemEditorFactory * factory = new QItemEditorFactory();
            this->setItemEditorFactory(factory);
            factory->registerEditor(QVariant::Int, new QItemEditorCreator<NeutralSpinBox>("value"));
            factory->registerEditor(QVariant::UInt, new QItemEditorCreator<NeutralSpinBox>("value"));
            factory->registerEditor(QVariant::Double, new QItemEditorCreator<BoundedFloatSpinBox>("value"));
            factory->registerEditor((QVariant::Type)qMetaTypeId<float>(), new QItemEditorCreator<BoundedFloatSpinBox>("value"));
            factory->registerEditor((QVariant::Type)qMetaTypeId<BoundedFloat>(), new QItemEditorCreator<BoundedFloatSpinBox>("boundedValue"));
        }

        void paint(QPainter *painter, const QStyleOptionViewItem &option,
                                   const QModelIndex &index) const
        {

            QVariant min = index.data(NodeItemModel::MinValue);
            QVariant max = index.data(NodeItemModel::MaxValue);

            if (index.column() == 1 && min.isValid() && max.isValid()) {
                int progress = index.data().toInt();

                const NumericNodeBase * node = static_cast<const NumericNodeBase * >(index.internalPointer());

                QVariant neutral = index.data(NodeItemModel::NeutralValue);

                StyleOptionNeutralBar progressBarOption;
                progressBarOption.rect = option.rect;
                progressBarOption.progress = progress;
                progressBarOption.text = QString::number(progress).append(QString::fromStdString(node->getUnits()));
                progressBarOption.textVisible = true;
                progressBarOption.maximum = max.toInt();
                progressBarOption.minimum = min.toInt();
                progressBarOption.neutral = neutral.isValid() ? neutral.toInt() : 0;

                QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                                   &progressBarOption, painter);
            } else
                QStyledItemDelegate::paint(painter, option, index);

        }

        void setEditorData(QWidget *editor, const QModelIndex &index) const{

            info() << "setting editor data for type" << index.data().typeName();//.value<BoundedFloat>();
            //if(BoundedFloatSpinBox*spin = dynamic_cast<BoundedFloatSpinBox*>(editor)){
            //    spin->setValue2(index.data().value<BoundedFloat>());
            //}

            if(!index.data().isValid()){
                error() << "invalid variant";
            }


            editor->setProperty(editor->metaObject()->userProperty().name(), index.data());

            QStyledItemDelegate::setEditorData(editor, index);

        }



        QWidget * createEditor ( QWidget * parent, const QStyleOptionViewItem & option,
                                 const QModelIndex & index ) const {
            QWidget * editor = QStyledItemDelegate::createEditor(parent, option, index);

            info() << "creating editor for type" << index.data().typeName();//.value<BoundedFloat>();

            setEditorData(editor, index);

            return editor;
        }
    };





    } // namespace gui
} // namespace cauv


#endif // CAUV_STYLE_H
