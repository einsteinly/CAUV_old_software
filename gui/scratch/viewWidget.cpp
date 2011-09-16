#include "viewWidget.h"

#include "elements/multiArc.h"
//#include "elements/arcSocket.h"
#include "elements/multiArcStart.h"
#include "elements/multiArcEnd.h"
#include "elements/style.h"

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
    
    MultiArcStart *from_1 = new MultiArcStart(Image_Arc_Style);
    from_1->setPos(-50, 0);
    s->addItem(from_1);

    MultiArcStart *from_2 = new MultiArcStart(Image_Arc_Style);
    from_2->setPos(-50, 30); 
    s->addItem(from_2);
    MultiArcEnd *to_2_1 = new MultiArcEnd(from_2->arc());
    to_2_1->setPos(70, 0); // parent coords

    MultiArcStart *from_3 = new MultiArcStart(Param_Arc_Style);
    from_3->setPos(-50, -30);
    s->addItem(from_3);
    MultiArcEnd *to_3_1 = new MultiArcEnd(from_3->arc());
    to_3_1->setPos(70, -5);

    MultiArcStart *from_4 = new MultiArcStart(Image_Arc_Style);
    from_4->setPos(-50, 50);
    s->addItem(from_4);
    MultiArcEnd *to_4_1 = new MultiArcEnd(from_4->arc());
    to_4_1->setPos(70, 5);
    MultiArcEnd *to_4_2 = new MultiArcEnd(from_4->arc());
    to_4_2->setPos(70, 25);

    connect(s, SIGNAL(changed(const QList<QRectF>&)),
            this, SLOT(updateScene(const QList<QRectF>&)));
}

