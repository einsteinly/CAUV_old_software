#include "viewWidget.h"

#include "elements/multiArc.h"
//#include "elements/arcSocket.h"
//#include "elements/arcEnd.h"

#include "cross.h"

using namespace cauv;
using namespace cauv::gui;

ViewWidget::ViewWidget(QWidget* parent)
    : QGraphicsView(parent){
    
    QGraphicsScene *s = new QGraphicsScene(this);

    s->setItemIndexMethod(QGraphicsScene::NoIndex);
    s->setSceneRect(-200,-200,400,400);

    setScene(s);

    setCacheMode(CacheBackground);
    setViewportUpdateMode(MinimalViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(1.0,1.0);
    setMinimumSize(400, 400);
    setWindowTitle("QGraphicsScene Element Test");
    
    Cross *from_1 = new Cross(10);
    from_1->setPos(-50, 0);
    s->addItem(from_1);

    Cross *from_2 = new Cross(10);
    from_2->setPos(-50, 30); 
    s->addItem(from_2);
    
    Cross *to_2_1 = new Cross(10);
    to_2_1->setPos(20, 30);
    s->addItem(to_2_1);

    Cross *from_3 = new Cross(10);
    from_3->setPos(-50, -30);
    s->addItem(from_3);
    Cross *to_3_1 = new Cross(10);
    to_3_1->setPos(20, -35);
    s->addItem(to_3_1);

    Cross *from_4 = new Cross(10);
    from_4->setPos(-50, 50);
    s->addItem(from_4);
    Cross *to_4_1 = new Cross(10);
    to_4_1->setPos(20, 55);
    s->addItem(to_4_1);
    Cross *to_4_2 = new Cross(10);
    to_4_2->setPos(20, 75);
    s->addItem(to_4_2);
    
    MultiArc *a1 = new MultiArc(from_1);
    MultiArc *a2 = new MultiArc(from_2, to_2_1);
    MultiArc *a3 = new MultiArc(from_3, to_3_1);
    MultiArc *a4 = new MultiArc(from_4, to_4_1);
    a4->addTo(to_4_2);

    s->addItem(a1);
    s->addItem(a2);
    s->addItem(a3);
    s->addItem(a4);
}

