/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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
#include <QTimer>

#include <boost/make_shared.hpp>

#include <liquid/button.h>

#include <utility/string.h>
#include <utility/time.h>
#include <generated/types/NodeType.h>

#include "elements/style.h"
#include "style.h"

#include "framework/nodescene.h"

#include "fluidity/fNode.h"
#include "fluidity/managedElement.h"
#include "fluidity/manager.h"
#include "fluidity/menu.h"

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui::f;
using namespace liquid;

#include <QHash>
template<typename T>
uint qHash(boost::shared_ptr<T> p){
    return qHash(p.get());
}


FView::FView(boost::shared_ptr<CauvNode> node,
             std::string const& pipeline_name,
             QWidget* parent)
    : liquid::LiquidView(parent),
      m_cauv_node(node),
      m_manager(),
      m_contextmenu_root(),
      m_scenerect_update_timer(NULL),
      m_mode(TopLevel){

    initMenu();

    NodeScene *s = new NodeScene(this, false); // !!! false = don't set global model node scene

    // !!! is this really what we want to do?
    // items aren't added or removed a lot, just updated
    s->setItemIndexMethod(QGraphicsScene::NoIndex);
    s->setSceneRect(-4000,-4000,8000,8000);

    setScene(s);
    m_manager = boost::make_shared<Manager>(s, m_cauv_node.get(), pipeline_name);
    m_manager->init();

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

    m_scenerect_update_timer = new QTimer(this);
    connect(m_scenerect_update_timer, SIGNAL(timeout()), this, SLOT(setSceneRectToContents()), Qt::QueuedConnection);

    _initInMode(m_mode);

    /*
    //w::Graph* g = new w::Graph(w::One_Minute, "A Graph");
    w::GraphConfig config = {30.0};
    w::Graph* g1 = new w::Graph(config);
    w::Graph* g2 = new w::Graph(config);
    w::Graph* g3 = new w::Graph(config);

    s->addItem(g1);
    s->addItem(g2);
    s->addItem(g3);

    g2->setPos(0, -450);
    g1->setPos(0, 0);
    g3->setPos(0, 170);

    m_pct_series = w::DataSeries_ptr(new w::DataSeries(w::Percent_Graph, "test_pct_graph"));
    m_pct2_series = w::DataSeries_ptr(new w::DataSeries(w::Percent_Graph, "test_pct2_graph"));
    m_unlim_series = w::DataSeries_ptr(new w::DataSeries(w::Unlimited_Graph, "test_unlim_graph"));
    m_angle_series = w::DataSeries_ptr(new w::DataSeries(w::Degrees_Angle_Graph, "test_angle_graph"));

    g1->setRect(QRectF(0,0,200,140));
    g1->addDataSeries(m_pct_series);
    g1->addDataSeries(m_pct2_series);
    
    g2->setRect(QRectF(0,0,600,400));
    g2->addDataSeries(m_unlim_series);
    g2->addDataSeries(m_angle_series);

    g3->setRect(QRectF(0,0,120,80));
    g3->addDataSeries(m_angle_series);
    g3->addDataSeries(m_pct_series);
    g3->addDataSeries(m_pct2_series);
    g3->addDataSeries(m_unlim_series);

    m_data_timer   = new QTimer(this);

    connect(m_data_timer, SIGNAL(timeout()), this, SLOT(postData()), Qt::QueuedConnection);
    
    m_data_timer->setInterval(25);
    m_data_timer->start();
    */
}

void FView::_initInMode(Mode const& mode){
    m_mode = mode;

    typedef std::pair<QPoint, QGraphicsWidget*> pt_widget_pair_t;
    
    if(mode == TopLevel){
        setMinimumSize(800, 600);
        foreach(pt_widget_pair_t const& oi, m_overlay_items){
            oi.second->show();
        }
        setInteractive(true);
        setDragMode(QGraphicsView::ScrollHandDrag);

        // QGraphicsView spends most of its time testing the intersection of
        // bounding boxes without this set, and in any case OpenGL doesn't
        // support partial updates:
        setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
        m_scenerect_update_timer->stop();

    }else{
        setMinimumSize(80, 60);
        foreach(pt_widget_pair_t const& oi, m_overlay_items){
            oi.second->hide();
        }
        setInteractive(false);
        setDragMode(QGraphicsView::NoDrag);

        // Fixed frame-rate
        setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
        m_scenerect_update_timer->setInterval(1000);
        m_scenerect_update_timer->start();
    }

}


void FView::setMode(Mode const& m){
    if(m_mode != m){
        _initInMode(m);
    }
}

/*#include <utility/time.h>
void FView::postData(){
    static int i = 0;
    static double walk = 0;
    const double n = nowDouble();
    m_pct_series->postData(50+50*sin(i++/5e3) + ((rand() % 20) - 10), n);
    if(!(i % 7))
        m_pct2_series->postData(20+10*sin(i++/3e2) + ((rand() % 10) - 5), n);
    if(!(i % 3))
        //m_unlim_series->postData(((i) % 200 - 100), n);
        m_unlim_series->postData(walk += ((rand() % 100) - 50)/10.0, n);
    m_angle_series->postData(i/2.0, n);
}*/

void FView::initMenu(){
    QAction_ptr_set actions;

    // !!! TODO: might? need to use a deleter on the QAction shared_ptr that
    // calls deleteLater() instead of simply delete

    for(int i = 0; i < NodeType::NumValues; i++){
        QString n = QString::fromStdString(mkStr() << NodeType::e(i));
        n.replace("NodeType::","");
        n += " Node";
        boost::shared_ptr<QAction> a = boost::make_shared<QAction>(n, this);
        a->setData(uint32_t(i));
        connect(a.get(), SIGNAL(triggered()), this, SLOT(menuActioned()));
        actions << a;
    }

    m_contextmenu_root.action.reset();
    m_contextmenu_root.group_name = "Create Node";
    m_contextmenu_root.kids.clear();
    initMenu(m_contextmenu_root, actions);
}

struct ContainsWord{
    ContainsWord(std::string const& word) : m_word(QString::fromStdString(word)){ }

    bool operator()(boost::shared_ptr<QAction>  a) const{
        return a->text().contains(m_word, Qt::CaseInsensitive);
    }

    QString m_word;
};

float FView::split(std::string const& word, QAction_ptr_set actions){
    std::size_t do_contain_word = std::count_if(actions.begin(), actions.end(), ContainsWord(word));
    return float(do_contain_word) / actions.size();
}

template<typename T, typename P>
QSet<T> passesPredicate(QSet<T> const& in, P const& pred){
    QSet<T> r;
    foreach(T const& t, in)
        if(pred(t))
            r << t;
    return r;
}

void FView::initMenu(MenuNode& parent, QAction_ptr_set actions){
    if(actions.size() == 1){
        parent.action = *actions.begin();
        assert(parent.kids.size() == 0);
        return;
    }

    static const char* split_words_init[] = {
        "Sonar", "File", "Split", "Combine", "Math", "Corner", /*"Int",*/ "Float",
        "String", "Keypoint", "Lines", "Input", "Output", "Camera",
        "Histogram", "Max", "Min", "Segment", "Mix", "Filter", "Copy", "Mix",
        "Clamp", "Crop"
    };
    std::vector<std::string> split_words;
    for(std::size_t i = 0; i < sizeof(split_words_init) / sizeof(char*); i++){
        split_words.push_back(split_words_init[i]);
    }
    
    const int submenu_split_min = 2;
    while(actions.size()){
        float best_split = 0;
        int best_split_count = 0;
        std::string best_split_word;
        foreach(std::string const& w, split_words){
            const float s = split(w, actions);
            debug(5) << "split" << s << s*actions.size() << w;
            if(std::fabs(s-0.5) < std::fabs(best_split-0.5)){
                best_split = s;
                best_split_word = w;
                best_split_count = s*(actions.size()+0.5);
            }
        }
        debug(5) << "best split" << best_split << best_split_count << best_split_word;
        if(best_split_count >= submenu_split_min &&
           actions.size() - best_split_count > submenu_split_min){
            QAction_ptr_set does_contain = passesPredicate(actions, ContainsWord(best_split_word));
            actions -= does_contain;
            boost::shared_ptr<MenuNode> m = boost::make_shared<MenuNode>();
            m->group_name = QString::fromStdString(best_split_word);
            initMenu(*m, does_contain);
            parent.kids << m;
        }else{
            break;
        }
    }
    // add remaining actions (didn't fall into any subcategories) at this top
    // level:
    foreach(QAction_ptr p, actions){
        boost::shared_ptr<MenuNode> m = boost::make_shared<MenuNode>();
        m->action = p;
        parent.kids << m;
    }
}

void FView::_buildMenu(cauv::gui::f::Menu* menu, MenuNode const& node){
    if(node.action){
        menu->addAction(node.action.get());
    }else{
        cauv::gui::f::Menu* submenu = new cauv::gui::f::Menu(node.group_name, menu);
        menu->addMenu(submenu);
        foreach(boost::shared_ptr<MenuNode> kid, node.kids)
            _buildMenu(submenu, *kid);
    }
}

void FView::menuActioned(){
    QAction* s = dynamic_cast<QAction*>(sender());
    if(s){
        QVariant data = s->data();
        m_manager->requestNode(NodeType::e(data.value<uint32_t>()));
        debug() << "menuActioned:" << NodeType::e(data.value<uint32_t>());
    }else{
        error() << "menuActioned: non-action sender?";
    }
}

void FView::setSceneRectToContents(){
    QList<QGraphicsItem*> nodes = m_manager->rootNodes();
    
    if(nodes.size()){
        QRectF nodes_bounding_rect = nodes.at(0)->mapToScene(nodes.at(0)->boundingRect()).boundingRect();
        for(int i = 0; i < nodes.size(); i++){
            nodes_bounding_rect |= nodes.at(i)->mapToScene(nodes.at(i)->boundingRect()).boundingRect();
        }
        fitInView(nodes_bounding_rect.adjusted(-20, -20, 20, 20), Qt::KeepAspectRatio);

        // enforce maximum scale of 0.75:1
        QTransform tr = transform();
        const qreal det = tr.determinant();
        if(det > 0.75)
            tr.scale(0.75/det, 0.75/det);
        setTransform(tr);
    }
    
    update();
}


void FView::contextMenuEvent(QContextMenuEvent *event){
    cauv::gui::f::Menu menu(this);
    _buildMenu(&menu, m_contextmenu_root);
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
    // looks awful :(
    _updateOverlays();
    LiquidView::paintEvent(event);
}

// !!! temporary keyboard shortcut hack
void FView::keyPressEvent(QKeyEvent *event){
    LiquidView::keyPressEvent(event);
    if(!event->isAccepted()){
        event->accept();
        switch(event->key()){
            case Qt::Key_R:
                m_manager->requestRefresh();
                break;
            default:
                event->ignore();
                break;
        }
    }
}

void FView::mouseMoveEvent(QMouseEvent *event){
    m_manager->delayLayout();
    LiquidView::mouseMoveEvent(event);
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
        const QPointF new_pos = mapToScene(x, y);
        if((i->second->pos() - new_pos).manhattanLength() > 0.1)
            i->second->setPos(new_pos);
        const float new_scale = std::sqrt(1.0/transform().determinant());
        if(std::fabs(i->second->scale() - new_scale) > 1e-3)
            i->second->setScale(new_scale);
    }
}

