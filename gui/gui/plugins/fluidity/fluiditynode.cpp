/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "fluiditynode.h"

#include <QGraphicsLinearLayout>
#include <QTimeLine>

#include <debug/cauv_debug.h>

#include <liquid/button.h>
#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/style.h>
#include <liquid/nodeHeader.h>

#include <model/nodeItemModel.h>

#include <elements/style.h>
#include <nodepicker.h>

#include "view.h"
#include "manager.h"

#include "plugin.h"

using namespace cauv;
using namespace cauv::gui;

FluidityNode::FluidityNode(const std::string id) :
    Node(id, nodeType<FluidityNode>()){
}

FluidityNode::~FluidityNode(){
}

std::string FluidityNode::fullPipelineName(){
    std::string pipelineName = nodeName();
    if(getParent()->type == nodeType<GroupingNode>()){
        std::string parentName = getParent()->nodeName();
        pipelineName = parentName.append("/").append(pipelineName);
    }
    return pipelineName;
}


LiquidFluidityNode::LiquidFluidityNode(boost::shared_ptr<FluidityNode> node, 
                                       boost::weak_ptr<CauvMainWindow> in_window,
                                       QGraphicsItem *parent)
    : ConnectedNode(node, AI_Node_Style(), parent),
      m_node(node),
      m_contents(nullptr),
      m_view(nullptr),
      m_source(new liquid::ArcSource(new FluiditySourceDelegate(node),
                                     new liquid::Arc(Image_Arc_Style()))),
      m_in_window(in_window),
      m_orginal_view_rect(),
      m_zoomed_view_rect(),
      m_maximising(false){

    setResizable(true);

    setTitle(QString::fromStdString(m_node->nodeName()));
    setInfo(QString::fromStdString(m_node->nodePath()));

    //m_source->setParentItem(this);
    //m_source->setZValue(10);
    this->setClosable(false);
    
    std::string pipelineName = m_node->fullPipelineName();

    m_view = new f::FView(pipelineName, node);
    m_view->setMode(f::FView::Internal);
    m_view->setMinimumSize(120, 120);
    
    m_contents = new liquid::ProxyWidget(this);
    m_contents->setWidget(m_view);
    m_contents->setMinimumSize(120, 120);

    auto  hlayout = new QGraphicsLinearLayout(Qt::Horizontal);
    hlayout->setSpacing(0);
    hlayout->setContentsMargins(0,0,0,0);
    hlayout->addStretch(1);
    hlayout->addItem(m_contents);
    //hlayout->addItem(m_source);
    //hlayout->setAlignment(m_source, Qt::AlignBottom | Qt::AlignRight);

    addItem(hlayout);
    
    liquid::Button *maxbutton = new liquid::Button(
                QRectF(0,0,24,24), QString(":/resources/icons/maximise_button"), nullptr, this
                );
    addButton("maximise", maxbutton);
    
    connect(maxbutton, SIGNAL(pressed()), this, SLOT(beginMaximise()));
    connect(this, SIGNAL(doubleClicked()), this, SLOT(beginMaximise()));    

    setSize(QSizeF(300, 300));

    //connect(this, SIGNAL(xChanged()), m_source, SIGNAL(xChanged()));
    //connect(this, SIGNAL(yChanged()), m_source, SIGNAL(yChanged()));
}

void LiquidFluidityNode::beginMaximise(){
    if(m_maximising)
        return;
    m_maximising = true;
    QGraphicsScene* s = scene();
    if(!s)
        return maximise();
    QList<QGraphicsView*> views = s->views();
    if(!views.size())
        return maximise();

    m_orginal_view_rect = views[0]->mapToScene(views[0]->rect()).boundingRect();

    auto  timeline = new QTimeLine(800, this);
    timeline->setFrameRange(0, 100);
    connect(timeline, SIGNAL(frameChanged(int)), this, SLOT(zoomIn(int)));
    connect(timeline, SIGNAL(finished()), this, SLOT(maximise()));
    timeline->start();
}

void LiquidFluidityNode::zoomIn(int percent){
    float pct = percent/100.0f;
    QRectF target = m_contents->mapToScene(m_contents->boundingRect()).boundingRect();
    QRectF r(
                m_orginal_view_rect.x() + pct * (target.x()-m_orginal_view_rect.x()),
                m_orginal_view_rect.y() + pct * (target.y()-m_orginal_view_rect.y()),
                m_orginal_view_rect.width() + pct * (target.width()-m_orginal_view_rect.width()),
                m_orginal_view_rect.height() + pct * (target.height()-m_orginal_view_rect.height())
                );
    scene()->views()[0]->fitInView(r, Qt::KeepAspectRatio);
}

void LiquidFluidityNode::maximise(){
    boost::shared_ptr<CauvMainWindow> in_window = m_in_window.lock();
    if(in_window){
        m_contents->setWidget(nullptr);
        in_window->viewStack()->push(QString::fromStdString(m_node->nodeName()), m_view);
        m_view->setMode(f::FView::TopLevel);
        const float scale = std::sqrt(scene()->views()[0]->transform().determinant());
        m_view->scale(scale, scale);
        m_view->centerOn(m_contents->mapToScene(m_contents->boundingRect()).boundingRect().center());
        connect(m_view, SIGNAL(closeRequested()), this, SLOT(unMaximise()));
    }
    m_maximising = false;
}

void LiquidFluidityNode::zoomOut(int percent){
    float pct = percent/100.0f;
    QRectF target = m_orginal_view_rect;
    QRectF r(
                m_zoomed_view_rect.x() + pct * (target.x()-m_zoomed_view_rect.x()),
                m_zoomed_view_rect.y() + pct * (target.y()-m_zoomed_view_rect.y()),
                m_zoomed_view_rect.width() + pct * (target.width()-m_zoomed_view_rect.width()),
                m_zoomed_view_rect.height() + pct * (target.height()-m_zoomed_view_rect.height())
                );
    scene()->views()[0]->fitInView(r, Qt::KeepAspectRatio);
}

void LiquidFluidityNode::unMaximise(){
    m_zoomed_view_rect = scene()->views()[0]->mapToScene(scene()->views()[0]->rect()).boundingRect();

    boost::shared_ptr<CauvMainWindow> in_window = m_in_window.lock();
    if(in_window){
        in_window->viewStack()->pop();

        // !!! FIXME: for efficiency, we should re-use m_view, but this
        // segfaults (later, in an event handler) for some reason... Need a
        // debug build of Qt to tackle this, I think
        //m_view->setParent(0);
        //m_contents->setWidget(m_view);
        //m_view->show();
        //m_view->setMode(f::FView::Internal);

        // ... so create a new one instead:
        m_view->deleteLater();
        m_view = new f::FView(m_node->fullPipelineName(), m_view->manager()->model(), m_view->scene(), m_view->manager());
        m_view->setMode(f::FView::Internal);
        m_contents->setWidget(m_view);
    }

    auto  timeline = new QTimeLine(800, this);
    timeline->setFrameRange(0, 100);
    connect(timeline, SIGNAL(frameChanged(int)), this, SLOT(zoomOut(int)));
    timeline->start();
}


liquid::ArcSource *  LiquidFluidityNode::getSourceFor(boost::shared_ptr<Node> const&) const{
    return m_source;
}


boost::shared_ptr<FluidityNode> LiquidFluidityNode::fluidityNode(){
    return m_node;
}

