#ifndef NODESCENE_H
#define NODESCENE_H

#include <QGraphicsScene>
#include <QGraphicsSceneDragDropEvent>
#include <QWidget>

#include "drophandler.h"
#include "../nodedragging.h"

#include <gui/core/model/node.h>

namespace cauv {
    namespace gui {

        class NodeScene : public QGraphicsScene, public NodeDropListener {

        public:
            NodeScene(QObject * parent = NULL);

            // handlers
            virtual void registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > handler);
            QGraphicsItem * applyHandlers(boost::shared_ptr<NodeBase> node);
            bool hasHandlerFor(boost::shared_ptr<NodeBase> node);
            bool hasHandlerFor(std::vector<boost::shared_ptr<NodeBase> > nodes);

            // node dropping
            virtual void onNodeDropped(boost::shared_ptr<NodeBase> );
            void dropEvent(QGraphicsSceneDragDropEvent *event);
            void dragEnterEvent(QGraphicsSceneDragDropEvent *event);
            void dragMoveEvent(QGraphicsSceneDragDropEvent *event);

        protected:
            std::vector<boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > > m_handlers;
        };


        class ExampleDropHandler : public DropHandlerInterface<QGraphicsItem *> {

            virtual bool willHandle(boost::shared_ptr<NodeBase> node){
                return node->type == GuiNodeType::NumericNode;
            }

            virtual QGraphicsItem * handle(boost::shared_ptr<NodeBase> node) {
                throw drop_not_handled();
            }
        };

    } // namespace gui
} // namespace gui


#endif // NODESCENE_H
