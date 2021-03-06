/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ELEMENT_AI_NODES_H__
#define __CAUV_ELEMENT_AI_NODES_H__

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <model/nodes/numericnode.h>
#include <model/nodeItemModel.h>
#include <drag/nodeDragging.h>
#include <connectednode.h>

namespace cauv {
    class CauvNode;

    namespace gui {

        class AiNode : public ConnectedNode
        {
            Q_OBJECT
            typedef liquid::LiquidNode base_t;
        public:
            AiNode(boost::shared_ptr<Node> node, QGraphicsItem *parent=0);
            virtual ~AiNode();
        public Q_SLOTS:
            void close();
        };

        class AiDropHandler : public QObject, public DropHandlerInterface<QGraphicsItem * > {
            Q_OBJECT
        public:
            AiDropHandler(boost::shared_ptr<NodeItemModel> model, boost::weak_ptr<CauvNode> node);
            virtual bool accepts(boost::shared_ptr<Node> const& node);
            virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node);

        protected:
            boost::shared_ptr<NodeItemModel> m_model;
            boost::weak_ptr<CauvNode> m_node;
        };

    } // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_AI_NODES_H__
