#include "viewWidget.h"

#include <liquid/button.h>

#include "elements/multiArc.h"
//#include "elements/arcSocket.h"
#include "elements/multiArcStart.h"
#include "elements/multiArcEnd.h"
#include "elements/fNode.h"
#include "elements/style.h"

#include "fluidity/managedElement.h"
#include "fluidity/manager.h"

#include "cross.h"

using namespace cauv;
using namespace cauv::gui;
using namespace liquid;

ViewWidget::ViewWidget(QWidget* parent)
    : QGraphicsView(parent){
    
    QGraphicsScene *s = new QGraphicsScene(this);

    s->setItemIndexMethod(QGraphicsScene::NoIndex);
    s->setSceneRect(-200,-200,400,400);

    setScene(s);

    Manager m(s, NULL);

    FNode *n1 = new FNode(m);
    n1->setPos(20.5, 100.5);

    FNode *n2 = new FNode(m);
    n2->setPos(-50.5, -100.5);
    
    FNode *n3 = new FNode(m);
    n3->setPos(-70.5, -120.5);

    setCacheMode(CacheBackground);
    setViewportUpdateMode(MinimalViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(1.0,1.0);
    setMinimumSize(400, 400);
    setWindowTitle("QGraphicsScene Element Test");
    
    MultiArcStart *from_1 = new MultiArcStart(*n2, Image_Arc_Style);
    from_1->setPos(-50, 0);

    MultiArcStart *from_2 = new MultiArcStart(*n2, Image_Arc_Style);
    from_2->setParentItem(n2);
    from_2->setPos(104.5, 105);
    connect(n2, SIGNAL(xChanged()), from_2, SIGNAL(xChanged()));
    connect(n2, SIGNAL(yChanged()), from_2, SIGNAL(yChanged()));

    MultiArcEnd *to_2_1 = new MultiArcEnd(from_2->arc());
    to_2_1->setPos(70, 0); // parent coords


    MultiArcStart *from_3 = new MultiArcStart(*n3, Param_Arc_Style);
    from_3->setParentItem(n3);
    from_3->setPos(104.5, 105);
    connect(n3, SIGNAL(xChanged()), from_3, SIGNAL(xChanged()));
    connect(n3, SIGNAL(yChanged()), from_3, SIGNAL(yChanged()));

    MultiArcEnd *to_3_1 = new MultiArcEnd(from_3->arc());
    to_3_1->setPos(70, -5);


    s->addItem(n1);
    s->addItem(n2);
    s->addItem(n3);

    MultiArcStart *from_4 = new MultiArcStart(*n3, Image_Arc_Style);
    from_4->setParentItem(n3);
    from_4->setPos(104.5, 90);
    connect(n3, SIGNAL(xChanged()), from_4, SIGNAL(xChanged()));
    connect(n3, SIGNAL(yChanged()), from_4, SIGNAL(yChanged()));
    

    MultiArcEnd *to_4_1 = new MultiArcEnd(from_4->arc());
    to_4_1->setParentItem(n2);
    to_4_1->setFlag(QGraphicsItem::ItemStacksBehindParent);    
    to_4_1->setPos(-0.5,68); 
    connect(n2, SIGNAL(xChanged()), to_4_1, SIGNAL(xChanged()));
    connect(n2, SIGNAL(yChanged()), to_4_1, SIGNAL(yChanged()));

    MultiArcEnd *to_4_2 = new MultiArcEnd(from_4->arc());
    to_4_2->setParentItem(n1);
    to_4_2->setFlag(QGraphicsItem::ItemStacksBehindParent);
    to_4_2->setPos(-0.5,54);
    connect(n1, SIGNAL(xChanged()), to_4_2, SIGNAL(xChanged()));
    connect(n1, SIGNAL(yChanged()), to_4_2, SIGNAL(yChanged()));
    
    MultiArcEnd *to_4_3 = new MultiArcEnd(from_4->arc());
    to_4_3->setParentItem(n1);
    to_4_3->setFlag(QGraphicsItem::ItemStacksBehindParent);    
    to_4_3->setPos(-0.5,68);
    connect(n1, SIGNAL(xChanged()), to_4_3, SIGNAL(xChanged()));
    connect(n1, SIGNAL(yChanged()), to_4_3, SIGNAL(yChanged()));

    //s->addItem(from_1);
    //s->addItem(from_2);
    //s->addItem(from_3);
    
    Button *b;
    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/dup_button"));
    b->setPos(170,176);
    s->addItem(b);
    
    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/x_button"));
    b->setPos(146,176);
    s->addItem(b);
    
    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"));
    b->setPos(122,176);
    s->addItem(b);
    
    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"));
    b->setPos(98,176);
    s->addItem(b);
    
    connect(s, SIGNAL(changed(const QList<QRectF>&)),
            this, SLOT(updateScene(const QList<QRectF>&)));
}

