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

#include <QHash>
template<typename T>
uint qHash(boost::shared_ptr<T> p){
    return qHash(p.get());
}


FView::FView(boost::shared_ptr<CauvNode> node, QWidget* parent)
    : LiquidView(parent),
      m_cauv_node(node),
      m_manager(),
      m_contextmenu_root(){
    initMenu();

    QGraphicsScene *s = new QGraphicsScene(this);

    // !!! is this really what we want to do?
    // items aren't added or removed a lot, just updated
    s->setItemIndexMethod(QGraphicsScene::NoIndex);
    s->setSceneRect(-4000,-4000,8000,8000);

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
    // looks aweful :(
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

