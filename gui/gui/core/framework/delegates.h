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


#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QApplication>

namespace cauv {
    namespace gui {


    struct NodeDelegateEditorInterface {
        virtual void configureEditor(QWidget * widget, boost::shared_ptr<Node> node) = 0;
    };


    class NodeDelegateMapper : public QStyledItemDelegate
    {
        Q_OBJECT

    public:
        NodeDelegateMapper(QObject *parent = 0){
        }


        void registerDelegate(GuiNodeType::e nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate){
            m_delegates[nodeType] = delegate;
        }

        QWidget * createEditor ( QWidget * parent, const QStyleOptionViewItem & option,
                                 const QModelIndex & index ) const {

            Node* node = static_cast<Node*>(index.internalPointer());
            try {
                boost::shared_ptr<QAbstractItemDelegate> delegate = m_delegates.at(node->type);
                QWidget * w = delegate->createEditor(parent, option, index);
                NodeDelegateEditorInterface * editorDelegate = dynamic_cast<NodeDelegateEditorInterface *>(delegate.get());
                editorDelegate->configureEditor(w, node->shared_from_this());
                return w;
            } catch (std::out_of_range){
                return QStyledItemDelegate::createEditor(parent,option, index);
            }
        }


        void paint(QPainter *painter, const QStyleOptionViewItem &option,
                   const QModelIndex &index) const{

            // sort out list decoration
            if (!hasParent(index)) {
                // Paint the top-item
            } else if (isLast(index)) {
                // Paint the bottom item
            } else {
                // Paint middle items
            }

            // display the node
            Node * node = dynamic_cast<Node*>((Node*)index.internalPointer());
            if (node && index.column() == 1) {
                try {
                    boost::shared_ptr<QAbstractItemDelegate> delegate = m_delegates.at(node->type);
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

        std::map<GuiNodeType::e, boost::shared_ptr<QAbstractItemDelegate> > m_delegates;

    private:
        bool hasParent(const QModelIndex &index) const{
            if (index.parent().isValid())
                return true;

            return false;
        }

        bool isLast(const QModelIndex &index) const{
            if (index.parent().isValid())
                if (!index.parent().child(index.row()+1,
                                          index.column()).isValid())
                    return true;

            return false;
        }
    };



    struct ProgressBarDelegate : public QStyledItemDelegate, public NodeDelegateEditorInterface {

        ProgressBarDelegate(QObject * parent = 0) : QStyledItemDelegate(parent) {
            QItemEditorFactory * factory = new QItemEditorFactory();
            this->setItemEditorFactory(factory);
            factory->registerEditor(QVariant::Int, new QItemEditorCreator<NeutralSpinBox>("value"));
            factory->registerEditor(QVariant::UInt, new QItemEditorCreator<NeutralSpinBox>("value"));
        }

        void paint(QPainter *painter, const QStyleOptionViewItem &option,
                                   const QModelIndex &index) const
        {
            if (index.column() == 1) {
                int progress = index.data().toInt();

                const NumericNodeBase * node = static_cast<const NumericNodeBase * >(index.internalPointer());

                StyleOptionNeutralBar progressBarOption;
                progressBarOption.rect = option.rect;
                progressBarOption.progress = progress;
                progressBarOption.text = QString::number(progress).append(QString::fromStdString(node->getUnits()));
                progressBarOption.textVisible = true;
                progressBarOption.maximum = node->getMax().toInt();
                progressBarOption.minimum = node->getMin().toInt();
                progressBarOption.neutral = 0;

                QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                                   &progressBarOption, painter);
            } else
                QStyledItemDelegate::paint(painter, option, index);

        }

        void configureEditor(QWidget * widget, boost::shared_ptr<Node> node){
            switch (node->type) {
            case GuiNodeType::NumericNode: {
                boost::shared_ptr<NumericNodeBase> numericNode =
                        boost::static_pointer_cast<NumericNodeBase>(node);

                QSpinBox * spin = qobject_cast<QSpinBox *>(widget);
                if(spin){
                    spin->setMaximum(numericNode->getMax().toInt());
                    spin->setMinimum(numericNode->getMin().toInt());
                }

                QDoubleSpinBox * doubleSpin = qobject_cast<QDoubleSpinBox *>(widget);
                if(doubleSpin){
                    doubleSpin->setMaximum(numericNode->getMax().toDouble());
                    doubleSpin->setMinimum(numericNode->getMin().toDouble());
                }
            } break;
            default:
                warning() << "NodeDelegate tried to create editor for unsupported node type";
            }
        }
    };





    } // namespace gui
} // namespace cauv


#endif // CAUV_STYLE_H
