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

#include <gui/core/model/model.h>
#include <gui/core/model/nodes/numericnode.h>

#include <QPainter>

#include <QStyledItemDelegate>
#include <QItemEditorFactory>
#include <QItemEditorCreator>
#include <QSpinBox>


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

            void registerDelegate(node_type nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate);

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
