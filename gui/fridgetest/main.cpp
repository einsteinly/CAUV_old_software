/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QApplication>

#include <liquid/itemFridge.h>

class TestGraphicsItem2: public QGraphicsItem{
    public:
        TestGraphicsItem2(QGraphicsItem* parent=0)
            : QGraphicsItem(parent),
              m_outline(0,0,40,40),
              m_ellipse(new QGraphicsEllipseItem(0, 0, 20, 16, this)),
              m_rect(new QGraphicsRectItem(0, 0, 16, 24, this)){

            m_ellipse->setPos(0, -10);
            m_ellipse->setPen(QPen(QColor(10,100,30,128)));
            m_ellipse->setBrush(QBrush(QColor(20,120,40,120)));

            m_rect->setPos(10,20);
            m_rect->setPen(QPen(QColor(10,30,120,128)));
            m_rect->setBrush(QBrush(QColor(20,30,160,120)));

            setCacheMode(QGraphicsItem::ItemCoordinateCache);
            m_ellipse->setCacheMode(QGraphicsItem::ItemCoordinateCache);
            m_rect->setCacheMode(QGraphicsItem::ItemCoordinateCache);
        }
        

        virtual QRectF	boundingRect() const{
            return m_outline;
        }

        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* o, QWidget* w=0){
            //std::cout << "TestGraphicsItem2::paint!" << std::endl;
            Q_UNUSED(o);
            Q_UNUSED(w);
            painter->setPen(QPen(QColor(120,20,20,120)));
            painter->setBrush(QBrush(QColor(100,10,10,64)));
            painter->drawRect(m_outline);
        }

    private:
        QRectF m_outline;
        QGraphicsEllipseItem* m_ellipse;
        QGraphicsRectItem* m_rect;
};

class TestGraphicsItem: public QGraphicsItem{
    public:
        TestGraphicsItem(QGraphicsItem* parent=0)
            : QGraphicsItem(parent),
              m_outline(0,0,40,40),
              m_ellipse(new QGraphicsEllipseItem(0, 0, 30, 40, this)),
              m_rect(new QGraphicsRectItem(0, 0, 20, 20, this)),
              m_complex_child(new TestGraphicsItem2(this)){

            m_ellipse->setPos(10, 10);
            m_ellipse->setPen(QPen(QColor(10,100,30,255)));
            m_ellipse->setBrush(QBrush(QColor(20,120,40,240)));

            m_rect->setPos(-10,20);
            m_rect->setPen(QPen(QColor(10,30,120,255)));
            m_rect->setBrush(QBrush(QColor(20,30,160,240)));

            m_complex_child->setPos(14,14);
            m_complex_child->setFlag(QGraphicsItem::ItemStacksBehindParent);
            
            setCacheMode(QGraphicsItem::ItemCoordinateCache);
            m_ellipse->setCacheMode(QGraphicsItem::ItemCoordinateCache);
            m_rect->setCacheMode(QGraphicsItem::ItemCoordinateCache);
            m_complex_child->setCacheMode(QGraphicsItem::ItemCoordinateCache);
        }
        

        virtual QRectF	boundingRect() const{
            return m_outline;
        }

        virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* o, QWidget* w=0){
            //std::cout << "TestGraphicsItem::paint!" << std::endl;
            Q_UNUSED(o);
            Q_UNUSED(w);
            painter->setPen(QPen(QColor(120, 20, 20, 230)));
            painter->setBrush(QBrush(QColor(100,10,10,128)));
            painter->drawRect(m_outline);
        }

    private:
        QRectF m_outline;
        QGraphicsEllipseItem* m_ellipse;
        QGraphicsRectItem* m_rect;
        TestGraphicsItem2* m_complex_child;
};

int main(int argc, char *argv[]){
    using liquid::ItemFridge;

    QApplication app(argc, argv);
 
    QGraphicsScene scene;
    scene.setSceneRect(-100.0, -100.0, 200.0, 200.0);
 
    auto item = new ItemFridge<TestGraphicsItem>();
    //TestGraphicsItem *item = new TestGraphicsItem(0);
    scene.addItem(item);
    item->setFlag(QGraphicsItem::ItemIsMovable);
    item->freeze();
    item->setPos(1,1);
 
    QGraphicsView view(&scene);
    view.setRenderHints(QPainter::Antialiasing);
    view.show();
    view.scale(1,1);
 
    return app.exec();
}
