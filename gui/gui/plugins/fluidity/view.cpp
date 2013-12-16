/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "view.h"

#include <QDebug>
#include <QGraphicsRectItem>
#include <QAction>
#include <QTimer>
#include <QGLWidget>

#include <boost/make_shared.hpp>

#include <liquid/button.h>

#include <utility/string.h>
#include <utility/time.h>

#include <model/node.h>
#include <elements/style.h>
#include <styles/style.h>

#include <nodescene.h>

#include "fNode.h"
#include "managedElement.h"
#include "manager.h"
#include "menu.h"

#include <debug/cauv_debug.h>

using namespace cauv;
using namespace cauv::gui::f;
using namespace liquid;

#include <QHash>
template<typename T>
uint qHash(boost::shared_ptr<T> p){
    return qHash(p.get());
}


FView::FView(const std::string& pipeline_name,
             boost::shared_ptr<gui::Node> model_parent,
             QWidget* parent)
    : liquid::LiquidView(parent),
      m_manager(),
      m_contextmenu_root(),
      m_scenerect_update_timer(nullptr),
      m_mode(TopLevel){

    init(pipeline_name, model_parent, nullptr, boost::shared_ptr<Manager>(), parent);
}

FView::FView(const std::string& pipeline_name,
             boost::shared_ptr<gui::Node> model_parent,
             NodeScene* s,
             boost::shared_ptr<Manager> m,
             QWidget* parent)
    : liquid::LiquidView(parent),
      m_manager(),
      m_contextmenu_root(),
      m_scenerect_update_timer(nullptr),
      m_mode(TopLevel){
    
    init(pipeline_name, model_parent, s, m, parent);
}

void FView::init(const std::string& pipeline_name,
                 boost::shared_ptr<gui::Node> model_parent,
                 NodeScene* s,
                 boost::shared_ptr<Manager> m,
                 QWidget* parent){
    m_manager = m;

    if(!s){
        s = new NodeScene(parent);
        // !!! is this really what we want to do?
        // items aren't added or removed a lot, just updated
        s->setItemIndexMethod(QGraphicsScene::NoIndex);
        s->setSceneRect(-4000,-4000,8000,8000);
    }
    setScene(s);

    if(!m_manager){
        m_manager = boost::make_shared<f::Manager>(
            scene(), model_parent, pipeline_name
        );
        m_manager->init();
    }

    initMenu();

    setWindowTitle("Fluidity");

    Button *b;

    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/x_button"));
    s->addItem(b);
    b->setZValue(1000);
    b->setToolTip(QString("Close pipeline editor"));
    m_overlay_items.push_back(std::make_pair(QPoint(-40, -80), b));
    connect(b, SIGNAL(pressed()), this, SIGNAL(closeRequested()));

    b = new Button(QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"));
    b->setToolTip(QString("Reload pipeline"));
    s->addItem(b);
    b->setZValue(1000);
    m_overlay_items.push_back(std::make_pair(QPoint(-40, -40), b));
    connect(b, SIGNAL(pressed()), m_manager.get(), SLOT(requestRefresh()));

    m_scenerect_update_timer = new QTimer(this);
    connect(m_scenerect_update_timer, SIGNAL(timeout()), this, SLOT(setSceneRectToContents()), Qt::QueuedConnection);


#ifdef QT_PROFILE_GRAPHICSSCENE
    QTimer* profile_dump_timer = new QTimer(this);
    connect(profile_dump_timer, SIGNAL(timeout()), this, SLOT(dumpProfile()), Qt::QueuedConnection); 
    profile_dump_timer->setInterval(10000);
    profile_dump_timer->start();
#endif // def QT_PROFILE_GRAPHICSSCENE

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

FView::~FView(){
    CAUV_LOG_DEBUG(1, "~FView()");
    std::vector< std::pair<QPoint, QGraphicsWidget*> >::iterator i;
    for(i = m_overlay_items.begin(); i != m_overlay_items.end(); i++){
        if(i->second->scene())
            i->second->scene()->removeItem(i->second);
        i->second->deleteLater();
    }
}


void FView::_initInMode(Mode const& mode){
    m_mode = mode;

    typedef std::pair<QPoint, QGraphicsWidget*> pt_widget_pair_t;
    
    if(mode == TopLevel){
        setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
        setRenderHints(
            QPainter::Antialiasing |
            //QPainter::HighQualityAntialiasing |
            QPainter::TextAntialiasing |
            QPainter::SmoothPixmapTransform
        );

        setMinimumSize(800, 600);
        foreach(pt_widget_pair_t const& oi, m_overlay_items){
            oi.second->show();
        }
        setInteractive(true);
        setDragMode(QGraphicsView::ScrollHandDrag);

        // QGraphicsView spends most of its time testing the intersection of
        // bounding boxes without this set, and in any case OpenGL doesn't
        // support partial updates:
        //setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
        setViewportUpdateMode(QGraphicsView::MinimalViewportUpdate);
        m_scenerect_update_timer->stop();

         initMenu();
    }else{
        setViewport(nullptr);
        setRenderHints(
            QPainter::Antialiasing |
            QPainter::TextAntialiasing |
            QPainter::NonCosmeticDefaultPen
        );

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

        initMenu();
    }

}


void FView::setMode(Mode const& m){
    if(m_mode != m){
        _initInMode(m);
    }
}

cauv::gui::NodeScene* FView::scene(){
    QGraphicsScene* s = QGraphicsView::scene();
    NodeScene* r = dynamic_cast<NodeScene*>(s);
    assert(r);
    return r;
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

    m_contextmenu_root.action.reset();
    m_contextmenu_root.group_name = "Create Node";
    m_contextmenu_root.kids.clear();

    std::vector<std::string> test_names = {
        "test one",
        "test two",
        "foo",
        "bar",
        "baz"
    };

    int n_names = 0;
    for (auto& name: test_names) {
        pipeline_model::NodeModelType test_type(name);
        auto float_param = pipeline_model::FloatParam(4.0);
        auto int_param = pipeline_model::IntParam(4);
        test_type.addOutput("float_output", "test value", float_param);
        test_type.addOutput("int_output", "test value", int_param);
        test_type.addInput("float_input", "test value", float_param);
        test_type.addInput("int_input", "test value", int_param);
        pipeline_model::NodeModelType::addType(test_type);

        QString n = QString::fromStdString(name);
        boost::shared_ptr<QAction> a = boost::make_shared<QAction>(n, this);
        a->setData(n);
        connect(a.get(), SIGNAL(triggered()), this, SLOT(menuActioned()));
        boost::shared_ptr<MenuNode> m = boost::make_shared<MenuNode>();
        m->action = a;
        m_contextmenu_root.kids << m;
        actions << a;
    }

#if 0
//for later reference:
            boost::shared_ptr<MenuNode> m = boost::make_shared<MenuNode>();
            m->group_name = QString::fromStdString(best_split_word);
            initMenu(*m, does_contain);
            parent.kids << m;
#endif

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
    if (!s) { 
        CAUV_LOG_ERROR("menuActioned: non-action sender?");
        return;
    }
    try {
        QVariant data = s->data();
        std::string node_type = data.value<QString>().toStdString();
        m_manager->addNode(node_type);
        CAUV_LOG_DEBUG(2, "Added fluidity node of type" << node_type);
    } catch (pipeline_model::NoSuchNodeTypeException &e) {
        CAUV_LOG_ERROR(e.what());
    } 
}

void FView::setSceneRectToContents(){
    QList<QGraphicsItem*> nodes = m_manager->rootNodes();
    
    if(nodes.size()){
        QRectF nodes_bounding_rect = nodes.at(0)->mapToScene(nodes.at(0)->boundingRect()).boundingRect();
        for(auto & node : nodes){
            nodes_bounding_rect |= node->mapToScene(node->boundingRect()).boundingRect();
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


#ifdef QT_PROFILE_GRAPHICSSCENE
void FView::dumpProfile(){
    QMap<QString, quint64> profile_map = scene()->profileMap();

    QFile file("/tmp/com.cauv.qgraphicsscene.profile");
    file.open(QIODevice::WriteOnly | QIODevice::Text);

    QTextStream out(&file);
    out << "Item,Cumulative Ticks\n"; 
    for(auto i = profile_map.begin(); i != profile_map.end(); i++){
        out << i.key() << ", " << i.value() << '\n';
    }
}
#endif // def QT_PROFILE_GRAPHICSSCENE

void FView::contextMenuEvent(QContextMenuEvent *event){
    cauv::gui::f::Menu menu(this);
    _buildMenu(&menu, m_contextmenu_root);
    menu.exec(event->globalPos());
}

void FView::paintEvent(QPaintEvent * event){
    // hooking on resize/scroll events updates the positions too late and it
    // looks awful :(
    _updateOverlays();
    LiquidView::paintEvent(event);
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
        CAUV_LOG_DEBUG(8, "_updateOverlays:" << i->first.x() << i->first.y() << "->" << x << y);
        const QPointF new_pos = mapToScene(x, y);
        if((i->second->pos() - new_pos).manhattanLength() > 0.1)
            i->second->setPos(new_pos);
        const float new_scale = std::sqrt(1.0/transform().determinant());
        if(std::fabs(i->second->scale() - new_scale) > 1e-3)
            i->second->setScale(new_scale);
    }
}

