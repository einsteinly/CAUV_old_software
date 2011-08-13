#include "nodevisualiser.h"

#include <debug/cauv_debug.h>

#include "../model/node.h"

#include <QApplication>
#include <QGraphicsProxyWidget>

using namespace cauv;
using namespace cauv::gui;

class ZoomFilter : public QObject {
    bool eventFilter(QObject *object, QEvent *event)
    {

        if (event->type() == QEvent::KeyRelease) {
            QKeyEvent *keyEvent = dynamic_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape) {
                if(NodeVisualiser * vis = dynamic_cast<NodeVisualiser *>(object)){
                    vis->resetMatrix();
                }
                // let the event propage
            }
        }

        /*
        if (event->type() == QEvent::Wheel) {
            if(NodeVisualiser * vis = dynamic_cast<NodeVisualiser *>(object)){
                QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
                vis->centerOn(vis->mapToScene(wheelEvent->x(), wheelEvent->y()));

                if(wheelEvent->delta() > 0) {
                    vis->scale(vis->scaleFactor(), vis->scaleFactor());
                } else {
                    vis->scale(1.0 / vis->scaleFactor(), 1.0 / vis->scaleFactor());
                }
                event->accept();
            }
            return true;
        }*/
        return false;
    }
};



NodeVisualiser::NodeVisualiser(QWidget * parent) : QGraphicsView(parent),
m_scaleFactor(1.3)
{
    this->setAcceptDrops(true);

    installEventFilter(new ZoomFilter());

    setDragMode(QGraphicsView::ScrollHandDrag);

    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setFocusPolicy(Qt::WheelFocus);

}

void NodeVisualiser::wheelEvent(QWheelEvent *event){
    debug(6) << "wheelEvent" << this;

    event->ignore();

    // The passing of events between QGraphicsView and a QGraphicsView embedded
    // in a QGraphicsScene seems to have a bug where the accepted flag is not set
    // correctly, so this has to be hacked around a bit to achieve what we're after
    // ideally we'd just call QGraphicsView::wheelEvent(event) to propagate events
    // instead we check to see if the item below the cursor is a proxied NodeVisualiser
    // and pass the event on directly
    QGraphicsItem * item = itemAt(event->x(), event->y());
    if(QGraphicsProxyWidget * proxy = dynamic_cast<QGraphicsProxyWidget *>(item)){
        if(NodeVisualiser * vis = dynamic_cast<NodeVisualiser *>(proxy->widget())){
            vis->wheelEvent(event);
        }
    }
    
    if(!event->isAccepted()) {
        centerOn(mapToScene(event->x(), event->y()));
    
        if(event->delta() > 0) {
            scale(scaleFactor(), scaleFactor());
        } else {
            scale(1.0 / scaleFactor(), 1.0 / scaleFactor());
        }
        event->accept();
    }

    QGraphicsView::wheelEvent(event);
}

float NodeVisualiser::scaleFactor(){
    return m_scaleFactor;
}

void NodeVisualiser::setScaleFactor(float scaleFactor){
    m_scaleFactor = scaleFactor;
}
