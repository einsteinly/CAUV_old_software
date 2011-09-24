#include "view.h"

#include <QGraphicsProxyWidget>
#include <QGLWidget>

using namespace liquid;

class DragModeFilter : public QObject {
    bool eventFilter(QObject *object, QEvent *event)
    {
        if (event->type() == QEvent::KeyRelease) {
            QKeyEvent *keyEvent = dynamic_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Control) {
                if(LiquidView * vis = dynamic_cast<LiquidView *>(object)){
                    vis->setDragMode(QGraphicsView::ScrollHandDrag);
                }
                return true;
            }
        }

        if (event->type() == QEvent::KeyPress) {
            QKeyEvent *keyEvent = dynamic_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Control) {
                if(LiquidView * vis = dynamic_cast<LiquidView *>(object)){
                    vis->setDragMode(QGraphicsView::RubberBandDrag);
                }
                return true;
            }
        }

        return false;
    }
};



class ZoomFilter : public QObject {
    bool eventFilter(QObject *object, QEvent *event)
    {

        if (event->type() == QEvent::KeyRelease) {
            QKeyEvent *keyEvent = dynamic_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape) {
                if(LiquidView * vis = dynamic_cast<LiquidView *>(object)){
                    vis->resetMatrix();
                    vis->centerOn(0,0);
                }
                // let the event propage
                return false;
            }
        }

        return false;
    }
};


LiquidView::LiquidView(QWidget * parent) : QGraphicsView(parent),
m_scaleFactor(1.25)
{
    this->setAcceptDrops(true);

    installEventFilter(new ZoomFilter());
    installEventFilter(new DragModeFilter());

    setDragMode(QGraphicsView::ScrollHandDrag);

    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setTransformationAnchor(LiquidView::AnchorViewCenter);
    
    centerOn(0,0);

    setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
}


void LiquidView::wheelEvent(QWheelEvent *event){

    event->ignore();

    // The passing of events between QGraphicsView and a QGraphicsView embedded
    // in a QGraphicsScene seems to have a bug where the accepted flag is not set
    // correctly, so this has to be hacked around a bit to achieve what we're after
    // ideally we'd just call QGraphicsView::wheelEvent(event) to propagate events
    // instead we check to see if the item below the cursor is a proxied NodeVisualiser
    // and pass the event on directly
    QGraphicsItem * item = itemAt(event->x(), event->y());
    if(QGraphicsProxyWidget * proxy = dynamic_cast<QGraphicsProxyWidget *>(item)){
        if(LiquidView * vis = dynamic_cast<LiquidView *>(proxy->widget())){
            vis->wheelEvent(event);
        }
    }
    
    if(!event->isAccepted()) {

        QPoint viewportCenter = QPoint(rect().width()>>1, rect().height()>>1);
        //qDebug() << "viewportCenter" << viewportCenter;
        QPointF screenCenter = mapToScene(viewportCenter);
        //qDebug() << "screenCenter" << screenCenter;


        //Get the position of the mouse before scaling, in scene coords
        QPointF pointBeforeScale(mapToScene(event->pos()));
        //qDebug() << "pointBeforeScale" << pointBeforeScale;

        if(event->delta() > 0) {
            scale(scaleFactor(), scaleFactor());
        } else {
            scale(1.0 / scaleFactor(), 1.0 / scaleFactor());
        }

        //Get the position after scaling, in scene coords
        QPointF pointAfterScale(mapToScene(event->pos()));
        //qDebug() << "pointAfterScale" << pointAfterScale;


        //Get the offset of how the screen moved
        QPointF offset = pointAfterScale - pointBeforeScale;
        //qDebug() << "offset" << offset;

        QPointF newCenter = screenCenter - offset;
        //qDebug() << "newCenter" << newCenter;
        centerOn(newCenter);

        event->accept();
    }

    //QGraphicsView::wheelEvent(event);
}

float LiquidView::scaleFactor(){
    return m_scaleFactor;
}

void LiquidView::setScaleFactor(float scaleFactor){
    m_scaleFactor = scaleFactor;
}
