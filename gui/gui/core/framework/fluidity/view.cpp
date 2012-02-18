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

#include "view.h"

#include <QDebug>
#include <QGraphicsRectItem>
#include <QAction>

#include <boost/make_shared.hpp>

#include <liquid/button.h>

#include <utility/string.h>
#include <generated/types/NodeType.h>

#include "elements/style.h"
#include "style.h"

#include "fluidity/fNode.h"
#include "fluidity/managedElement.h"
#include "fluidity/manager.h"
#include "fluidity/menu.h"

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui::f;
using namespace liquid;

FView::FView(boost::shared_ptr<CauvNode> node, QWidget* parent)
    : LiquidView(parent),
      m_cauv_node(node),
      m_manager(),
      m_contextmenu_actions(){
    buildMenus();

    QGraphicsScene *s = new QGraphicsScene(this);

    // !!! is this really what we want to do?
    // items aren't added or removed a lot, just updated
    //s->setItemIndexMethod(QGraphicsScene::NoIndex);
    s->setSceneRect(-200,-200,400,400);

    setScene(s);
    m_manager = boost::make_shared<Manager>(s, m_cauv_node.get(), "default");
    m_manager->init();

    setMinimumSize(800, 600);
    setWindowTitle("Fluidity");

    Button *b;
    //b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/dup_button"));
    //s->addItem(b);
    //b->setZValue(1000);
    //m_overlay_items.push_back(std::make_pair(QPoint(-112, -40), b));

    //b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"));
    //s->addItem(b);
    //b->setZValue(1000);
    //m_overlay_items.push_back(std::make_pair(QPoint(-88, -40), b));

    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/x_button"));
    s->addItem(b);
    b->setZValue(1000);
    m_overlay_items.push_back(std::make_pair(QPoint(-64, -40), b));

    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"));
    s->addItem(b);
    b->setZValue(1000);
    m_overlay_items.push_back(std::make_pair(QPoint(-40, -40), b));
    connect(b, SIGNAL(pressed()), m_manager.get(), SLOT(requestRefresh()));
}

void FView::buildMenus(){
    for(int i = 0; i < NodeType::NumValues; i++){
        QString n = QString::fromStdString(mkStr() << NodeType::e(i));
        n.replace("NodeType::","");
        n += "Node";
        QAction* a = new QAction(n, this);
        // ... TODO: connect signals
        m_contextmenu_actions << a;
    }
    /*
    QAction* separator = new QAction(this);
    separator->setSeparator(true);
    menu.addAction(separator);
    */
}

void FView::contextMenuEvent(QContextMenuEvent *event){
    cauv::gui::f::Menu menu(this);
    foreach(QAction* a, m_contextmenu_actions)
        menu.addAction(a);
    menu.exec(event->globalPos());
}

/*void FView::resizeEvent(QResizeEvent* event){
    debug() << "FView::resizeEvent";
    _updateOverlays();
    LiquidView::resizeEvent(event);
}

void FView::scrollContentsBy(int dx, int dy){
    debug() << "FView::scrollContentsBy" << dx << dy;
    _updateOverlays();
    LiquidView::scrollContentsBy(dx, dy);
}*/

void FView::paintEvent(QPaintEvent * event){
    // hooking on resize/scroll events updates the positions too late and it
    // looks aweful :(
    _updateOverlays();
    LiquidView::paintEvent(event);
}

// !!! temporary keyboard shortcut hack
void FView::keyPressEvent(QKeyEvent *event){
    switch(event->key()){
        case Qt::Key_R:
            m_manager->requestRefresh();
            return;
        default:
            LiquidView::keyPressEvent(event);
            return;
    }
}

void FView::_updateOverlays(){
    std::vector< std::pair<QPoint, QGraphicsWidget*> >::const_iterator i;
    for(i = m_overlay_items.begin(); i != m_overlay_items.end(); i++){
        int x, y;
        if(i->first.x() < 0)
            x = QWidget::rect().right() + i->first.x();
        else
            x = i->first.x();
        if(i->first.y() < 0)
            y = QWidget::rect().bottom() + i->first.y();
        else
            y = i->first.y();
        debug(8) << "_updateOverlays:" << i->first.x() << i->first.y() << "->" << x << y;
        i->second->setPos(mapToScene(x, y));
    }
}

