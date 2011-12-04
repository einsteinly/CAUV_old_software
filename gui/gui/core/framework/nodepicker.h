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

#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QTreeView>
#include <QKeyEvent>

#include <debug/cauv_debug.h>

#include "../model/model.h"


#include <QPainter>

#include <QStyledItemDelegate>

namespace Ui {
    class NodePicker;
}


namespace cauv {
    namespace gui {

        class NodeTreeItemBase;

        /**
          * Node filtering by entering a part of the path
          */
        class NodePathFilter : public QObject, public NodeFilterInterface {
            Q_OBJECT
        public:
            NodePathFilter(QObject * parent = NULL);

        public Q_SLOTS:
            void setText(QString const& string);
            QString getText();
            bool filter(boost::shared_ptr<Node> const& node);

        protected:
            QString m_text;
            bool containsText(boost::shared_ptr<Node> const& node);

        Q_SIGNALS:
            void filterChanged();
        };





        class NodeDelegateFactory
        {
        private:
          NodeDelegateFactory(){}

        public:
          static boost::shared_ptr<QAbstractItemDelegate> getDelegateFor(boost::shared_ptr<Node> node);
        };



        class NodeDelegate : public QStyledItemDelegate
        {
            Q_OBJECT

        public:
            NodeDelegate(QObject *parent = 0){
            }

            void paint(QPainter *painter,
                       const QStyleOptionViewItem &option,
                       const QModelIndex &index) const{

                // paint the background
                painter->setBrush(QBrush(QColor(200, 200, 200)));
                painter->setPen(QPen(QColor(200, 200, 200)));
                painter->drawRect(option.rect);

                // sort out list decoration
                if (!hasParent(index)) {
                    // Paint the top-item
                } else if (isLast(index)) {
                    // Paint the bottom item
                } else {
                    // Paint middle items
                }

                // display the node
                if (Node * node = dynamic_cast<Node*>((Node*)index.internalPointer())) {
                    boost::shared_ptr<QAbstractItemDelegate> delegate = NodeDelegateFactory::getDelegateFor(node->shared_from_this());
                    delegate->paint(painter, option, index);
                }
            }

            QSize sizeHint(const QStyleOptionViewItem &option,
                           const QModelIndex &index) const{
                return QSize(100, 30);
            }

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



        /**
          * Filterable tree view onto the node model
          */
        class NodeTreeView : public QTreeView {
            Q_OBJECT
        public:
            NodeTreeView(QWidget * parent = 0);
            virtual void registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter);

        private Q_SLOTS:
            void applyFilters();
            void applyFilters(QModelIndex const&);
            bool applyFilters(boost::shared_ptr<Node> const&);

        Q_SIGNALS:
            void onKeyPressed(QKeyEvent *event);

        protected:
            std::vector<boost::shared_ptr<NodeFilterInterface> > m_filters;
            void keyPressEvent(QKeyEvent *event);
        };


        /**
          * NodeTreeView with a filter entry box above it
          */
        class NodePicker : public QWidget {
            Q_OBJECT

        public:
            NodePicker(boost::shared_ptr<NodeItemModel> const& root);
            virtual ~NodePicker();

        protected Q_SLOTS:
            void redirectKeyboardFocus(QKeyEvent* key);

        protected:
            boost::shared_ptr<NodeItemModel> m_root;

        private:
            Ui::NodePicker *ui;
        };

    } // namespace gui
} // namespace cauv


#endif // DATASTREAMDISPLAYS_H
