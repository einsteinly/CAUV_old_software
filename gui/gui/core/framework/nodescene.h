#ifndef NODESCENE_H
#define NODESCENE_H

#include <QGraphicsScene>
#include <QGraphicsSceneDragDropEvent>
#include <QGraphicsItem>
#include <QWidget>
#include <QGraphicsProxyWidget>

#include "../nodedragging.h"

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>

#include "../widgets/graph.h"

namespace cauv {
    namespace gui {

        class NodeScene : public QGraphicsScene, public NodeDropListener {

        public:
            NodeScene(QObject * parent = NULL);

            // drop handlers
            virtual void registerDropHandler(boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > handler);
            QGraphicsItem * applyHandlers(boost::shared_ptr<NodeBase> node);
            // drop listener methods
            bool accepts(boost::shared_ptr<NodeBase>node);
            virtual void onNodeDroppedAt(boost::shared_ptr<NodeBase>, QPointF );

        protected:
            std::vector<boost::shared_ptr<DropHandlerInterface<QGraphicsItem *> > > m_handlers;

        };


        class ExampleDropHandler : public DropHandlerInterface<QGraphicsItem *> {

            virtual bool accepts(boost::shared_ptr<NodeBase> node){
                return node->type == GuiNodeType::NumericNode;
            }

            virtual QGraphicsItem * handle(boost::shared_ptr<NodeBase>) {
                QGraphicsRectItem * rect = new QGraphicsRectItem();
                rect->setRect(rect->x(), rect->y(), 100, 100);
                rect->setPen(QPen(Qt::red));
                rect->setBrush(QBrush(Qt::blue));
                rect->setFlag(QGraphicsItem::ItemIsMovable);
                rect->setFlag(QGraphicsItem::ItemIsSelectable);
                return rect;
            }
        };


        class GraphDropHandler : public DropHandlerInterface<QGraphicsItem *> {

            virtual bool accepts(boost::shared_ptr<NodeBase> node){
                return node->type == GuiNodeType::NumericNode;
            }

            virtual QGraphicsItem * handle(boost::shared_ptr<NodeBase> node) {
                GraphWidget * graph = new GraphWidget(boost::static_pointer_cast<NumericNode>(node));

                QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
                proxy->setWidget(graph);
                proxy->setFlag(QGraphicsItem::ItemIsMovable);
                proxy->setFlag(QGraphicsItem::ItemIsSelectable);
                proxy->installEventFilter(new NodeDropFilter(graph));
                return proxy;
            }
        };

    } // namespace gui
} // namespace gui


#endif // NODESCENE_H
