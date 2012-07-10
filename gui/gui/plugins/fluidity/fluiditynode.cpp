/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "fluiditynode.h"

#include <QGraphicsLinearLayout>
#include <QTimeLine>

#include <debug/cauv_debug.h>

#include <liquid/button.h>
#include <liquid/arcSink.h>
#include <liquid/arc.h>
#include <liquid/style.h>
#include <liquid/nodeHeader.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/model/model.h>
#include <gui/core/framework/nodepicker.h>
#include <gui/core/framework/fluidity/view.h>

#include "plugin.h"

using namespace cauv;
using namespace cauv::gui;


LiquidFluidityNode::LiquidFluidityNode(boost::shared_ptr<FluidityNode> node, 
                                       boost::weak_ptr<CauvMainWindow> in_window,
                                       QGraphicsItem *parent)
    : ConnectedNode(node, AI_Node_Style(), parent),
      m_node(node),
      m_contents(NULL),
      m_view(NULL),
      m_source(new liquid::ArcSource(new FluidtySourceDelegate(node),
                                     new liquid::Arc(Image_Arc_Style()))),
      m_in_window(in_window),
      m_orginal_view_rect(),
      m_zoomed_view_rect(){

    setResizable(true);

    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));

    m_source->setParentItem(this);
    m_source->setZValue(10);
    this->setClosable(false);
    
    boost::shared_ptr<CauvNode> cauv_node = FluidityPlugin::theCauvNode().lock();
    if(cauv_node){
        std::string pipelineName = m_node->nodeName();
        if(m_node->getParent()->type == nodeType<GroupingNode>()){
            std::string parentName = m_node->getParent()->nodeName();
            pipelineName = parentName.append("/").append(pipelineName);
        }

        m_view = new f::FView(cauv_node, pipelineName);
        m_view->setMode(f::FView::Internal);
        m_contents = new liquid::ProxyWidget(this);
        m_contents->setWidget(m_view);

        QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(Qt::Horizontal);
        hlayout->setSpacing(0);
        hlayout->setContentsMargins(0,0,0,0);
        hlayout->addStretch(1);
        hlayout->addItem(m_contents);
        hlayout->addItem(m_source);
        hlayout->setAlignment(m_source, Qt::AlignBottom | Qt::AlignRight);

        addItem(hlayout);
    }else{
        throw std::runtime_error("no cauv node available to init FluidityNode");
    }

    
    liquid::Button *maxbutton = new liquid::Button(
       QRectF(0,0,24,24), QString(":/resources/icons/maximise_button"), NULL, this
    );
    m_header->addButton("maximise", maxbutton);
    
    connect(maxbutton, SIGNAL(pressed()), this, SLOT(beginMaximise()));

    setSize(QSizeF(300, 300));

    connect(this, SIGNAL(xChanged()), m_source, SIGNAL(xChanged()));
    connect(this, SIGNAL(yChanged()), m_source, SIGNAL(yChanged()));
}

void LiquidFluidityNode::beginMaximise(){
    QGraphicsScene* s = scene();
    if(!s)
        return maximise();
    QList<QGraphicsView*> views = s->views();
    if(!views.size())
        return maximise();

    m_orginal_view_rect = views[0]->mapToScene(views[0]->rect()).boundingRect();

    QTimeLine *timeline = new QTimeLine(800, this);
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
        m_contents->setWidget(NULL);
        in_window->viewStack()->push(QString::fromStdString(m_node->nodeName()), m_view);
        m_view->setMode(f::FView::TopLevel);
        const float scale = std::sqrt(scene()->views()[0]->transform().determinant());
        m_view->scale(scale, scale);
        m_view->centerOn(m_contents->mapToScene(m_contents->boundingRect()).boundingRect().center());
        connect(m_view, SIGNAL(closeRequested()), this, SLOT(unMaximise()));
    }
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
        boost::shared_ptr<CauvNode> cauv_node = m_view->node();
        m_view->deleteLater();
        m_view = new f::FView(cauv_node, m_node->nodeName());
        m_view->setMode(f::FView::Internal);
        m_contents->setWidget(m_view);
    }

    QTimeLine *timeline = new QTimeLine(800, this);
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

