#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QTreeWidget>
#include <QKeyEvent>

#include <debug/cauv_debug.h>

#include "../nodedragging.h"
#include "../model/model.h"


namespace Ui {
    class DataStreamPicker;
}


namespace cauv {
    namespace gui {

        class NodeTreeItemBase;

        struct NodeListFilterInterface {

            // return false to filter out an item (and its children) from the list
            virtual bool filter(boost::shared_ptr<NodeBase> node) = 0;

            // should be implmented as a signal by subclasses
            virtual void filterChanged() = 0;
        };

        class NodePathFilter : public QObject, public NodeListFilterInterface {
            Q_OBJECT

        public Q_SLOTS:

            void setText(QString string);
            QString getText();
            bool filter(boost::shared_ptr<NodeBase>node);

        protected:
            QString m_text;
            bool containsText(boost::shared_ptr<NodeBase>node);

        Q_SIGNALS:
            void filterChanged();

        };

        class NodeListView : public QTreeWidget, public cauv::gui::NodeDragSource {
            Q_OBJECT
        public:
            NodeListView(QWidget * parent);

            std::vector<boost::shared_ptr<cauv::gui::NodeBase> > getDroppedNodes();
            virtual void registerListFilter(boost::shared_ptr<NodeListFilterInterface> filter);


        private Q_SLOTS:
            void editStarted(QTreeWidgetItem* item, int column);
            void itemEdited(QTreeWidgetItem* item, int column);
            void applyFilters();
            void applyFilters(NodeTreeItemBase *);
            bool applyFilters(boost::shared_ptr<NodeBase>);

        Q_SIGNALS:
            void onKeyPressed(QKeyEvent *event);

        protected:
            std::vector<boost::shared_ptr<NodeListFilterInterface> > m_filters;

            void keyPressEvent(QKeyEvent *event);
        };



        class NodePicker : public QWidget {
            Q_OBJECT

        public:
            NodePicker(boost::shared_ptr<AUV>auv);
            virtual ~NodePicker();

        protected Q_SLOTS:
            void redirectKeyboardFocus(QKeyEvent* key);

        protected:
            boost::shared_ptr<NodeTreeItemBase> m_root;

        private:
            Ui::DataStreamPicker *ui;
        };

    } // namespace gui
} // namespace cauv


#endif // DATASTREAMDISPLAYS_H
