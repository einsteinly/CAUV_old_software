/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "model/nodes/numericnode.h"
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

#include "nodeDragging.h"

using namespace cauv;
using namespace cauv::gui;


bool NodeDropListener::routeNode(boost::shared_ptr<Node> const& s, QPointF pos){
    CAUV_LOG_INFO("Routing stream" << s->nodeName());
    onNodeDroppedAt(s, pos);
    onNodeDropped(s);
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
                            boost::shared_ptr<Node> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dragEvent->acceptProposedAction();
                                dragEvent->accept();
                                return true;
                            }
                        } catch (std::runtime_error& ex){
                            CAUV_LOG_WARNING(ex.what());
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
                            boost::shared_ptr<Node> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dropEvent->acceptProposedAction();
                                dropEvent->accept();
                                m_listener->routeNode(node,  QPointF(dropEvent->pos()));
                            }
                        } catch (std::runtime_error& ex){
                            CAUV_LOG_WARNING(ex.what());
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
                            boost::shared_ptr<Node> node = VehicleRegistry::instance()->getNode(url);
                            if(m_listener->accepts(node)){
                                // accept the event as soon as we find one that we can handle
                                dndEvent->acceptProposedAction();
                                dndEvent->accept();
                                if(event->type() == QEvent::GraphicsSceneDrop) {
                                    m_listener->routeNode(node, dndEvent->scenePos());
                                }
                            }
                        } catch (std::runtime_error& ex){
                            CAUV_LOG_WARNING(ex.what());
                        } catch (std::exception& ex){
                            CAUV_LOG_ERROR(ex.what());
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


