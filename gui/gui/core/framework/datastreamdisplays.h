#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeWidget>
#include <QMdiArea>
#include <QKeyEvent>
#include <QDropEvent>
#include <QDragEnterEvent>

#include <debug/cauv_debug.h>

#include "../nodedragging.h"
#include "../model/model.h"

#include "drophandler.h"


namespace Ui {
    class DataStreamPicker;
}


namespace cauv {
    namespace gui {


        class NodeVisualisationArea : public QMdiArea, public NodeDropListener {

        public:
            NodeVisualisationArea(QWidget * parent = 0);
            void onNodeDropped(boost::shared_ptr<NumericNode> );
            void onNodeDropped(boost::shared_ptr<ImageNode> );
            void onNodeDropped(boost::shared_ptr<FloatYPRNode> );
            void onNodeDropped(boost::shared_ptr<FloatXYZNode> );
            void onNodeDropped(boost::shared_ptr<GroupingNode> );
            void dropEvent(QDropEvent * event);
            void dragEnterEvent(QDragEnterEvent * event);
            void addWindow(boost::shared_ptr<QWidget> content);

            void registerDropHandler(boost::shared_ptr<DropHandler> handler);

        protected:
            std::vector<boost::shared_ptr<DropHandler> > m_handlers;

            template<class T>
            void applyHandlers(boost::shared_ptr<T> node){
                BOOST_FOREACH(boost::shared_ptr<DropHandler> const& handler, m_handlers){
                    try {
                        handler->handle(node);
                        break; // only accept the first handler that matches
                    } catch (drop_not_handled){
                        debug(5) << "Handler not appropriate";
                    }
                }
            }
        };



        class NodeListView : public QTreeWidget, public cauv::gui::NodeDragSource {
            Q_OBJECT
        public:
            NodeListView(QWidget * parent);

            std::vector<boost::shared_ptr<cauv::gui::NodeBase> > getDroppedNodes();

            void keyPressEvent(QKeyEvent *event);

        private Q_SLOTS:
            void editStarted(QTreeWidgetItem* item, int column);
            void itemEdited(QTreeWidgetItem* item, int column);

        Q_SIGNALS:
            void onKeyPressed(QKeyEvent *event);
        };



        class NodePicker : public QDockWidget{
            Q_OBJECT

        public:
            NodePicker(boost::shared_ptr<AUV>auv);
            virtual ~NodePicker();

        protected Q_SLOTS:
            void redirectKeyboardFocus(QKeyEvent* key);

        private:
            Ui::DataStreamPicker *ui;
        };

    } // namespace gui
} // namespace cauv


#endif // DATASTREAMDISPLAYS_H
