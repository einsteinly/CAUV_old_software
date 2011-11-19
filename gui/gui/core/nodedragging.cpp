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

#include "model/nodes/numericnode.h"
#include "model/nodes/compoundnodes.h"
#include "model/nodes/imagenode.h"
#include "model/nodes/groupingnode.h"
#include "model/registry.h"

#include <stdexcept>

#include <debug/cauv_debug.h>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QGraphicsSceneDragDropEvent>
#include <QWidget>
#include <QUrl>
#include <QMimeData>

#include "nodedragging.h"

using namespace cauv;
using namespace cauv::gui;


bool NodeDropListener::routeNode(boost::shared_ptr<NodeBase> const& s, QPointF pos){

    info() << "Routing stream" << s->nodeName();

    onNodeDroppedAt(s, pos);
    onNodeDropped(s);

    switch (s->type){
    case GuiNodeType::NumericNode:
        onNodeDropped(boost::static_pointer_cast<NumericNode>(s));
        break;
    case GuiNodeType::ImageNode:
        onNodeDropped(boost::static_pointer_cast<ImageNode>(s));
        break;
    case GuiNodeType::FloatYPRNode:
        onNodeDropped(boost::static_pointer_cast<FloatYPRNode>(s));
        break;
    case GuiNodeType::FloatXYZNode:
        onNodeDropped(boost::static_pointer_cast<FloatXYZNode>(s));
        break;
    case GuiNodeType::GroupingNode:
        onNodeDropped(boost::static_pointer_cast<GroupingNode>(s));
        break;
    default:
        warning() << "Unknown node type dropped";
        return false;
    }

    return true;
}




NodeDropFilter::NodeDropFilter(NodeDropListener * listener) : m_listener(listener) {
}

bool NodeDropFilter::eventFilter(QObject *, QEvent *event)
{
    if ((event->type() == QEvent::DragEnter)) {
        QDragEnterEvent *dragEvent = dynamic_cast<QDragEnterEvent *>(event);

        if(dragEvent) {
            const QMimeData * mimeData = dragEvent->mimeData();
            if(mimeData->hasUrls()) {
                foreach (QUrl url, mimeData->urls()){
                    if(url.scheme() == "varstream") {
                        try {
                            boost::shared_ptr<NodeBase> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dragEvent->acceptProposedAction();
                                dragEvent->accept();
                                return true;
                            }
                        } catch (std::runtime_error ex){
                            warning() << ex.what();
                        }
                    }
                }
            }
        }
    }

    else if ((event->type() == QEvent::Drop)) {
        QDropEvent *dropEvent = dynamic_cast<QDropEvent *>(event);

        if (dropEvent) {
            const QMimeData * mimeData = dropEvent->mimeData();
            if(mimeData->hasUrls()) {
                foreach (QUrl url, mimeData->urls()){
                    if(url.scheme() == "varstream") {
                        try {
                            boost::shared_ptr<NodeBase> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dropEvent->acceptProposedAction();
                                dropEvent->accept();
                                m_listener->routeNode(node,  QPointF(dropEvent->pos()));
                            }
                        } catch (std::runtime_error ex){
                            warning() << ex.what();
                        }
                    }
                }
                return true;
            }
        }
    }

    else if (event->type() == QEvent::GraphicsSceneDragEnter  ||
             event->type() == QEvent::GraphicsSceneDragMove ||
             event->type() == QEvent::GraphicsSceneDrop) {
        QGraphicsSceneDragDropEvent *dndEvent = dynamic_cast<QGraphicsSceneDragDropEvent *>(event);

        if(dndEvent){
            const QMimeData * mimeData = dndEvent->mimeData();
            if(mimeData->hasUrls()) {
                foreach (QUrl url, mimeData->urls()){
                    if(url.scheme() == "varstream") {
                        try {
                            boost::shared_ptr<NodeBase> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dndEvent->acceptProposedAction();
                                dndEvent->accept();
                                if(event->type() == QEvent::GraphicsSceneDrop) {
                                    m_listener->routeNode(node, dndEvent->scenePos());
                                }
                            }
                        } catch (std::runtime_error ex){
                            warning() << ex.what();
                        }
                    }
                }
            }

            if(dndEvent->isAccepted()) return true;
        }
    }

    return false;
}


NodeSceneDropArea::NodeSceneDropArea(NodeDropListener * listener) : NodeDropFilter(listener) {
    this->setAcceptDrops(true);
}

bool NodeSceneDropArea::sceneEvent(QEvent *event) {
    return eventFilter(this, event);
}

void NodeSceneDropArea::updateGeometry(QRectF const& rect){
    this->setRect(rect);
}


