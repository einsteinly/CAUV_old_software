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
    : liquid::LiquidNode(AI_Node_Style(), parent),
      ManagedNode(this, node),
      m_node(node),
      m_contents(NULL),
      m_view(NULL),
      m_source(new liquid::ArcSource(this, new liquid::Arc(Image_Arc_Style()))),
      m_in_window(in_window){

    setResizable(true);

    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));

    m_source->setParentItem(this);
    m_source->setZValue(10);
    
    boost::shared_ptr<CauvNode> cauv_node = FluidityPlugin::theCauvNode().lock();
    if(cauv_node){
        m_view = new f::FView(cauv_node, m_node->nodeName());
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
    
    //connect(this, SIGNAL(closed(node_id_t const&)), &manager(), SLOT(requestRemoveNode(node_id_t const&)));
    connect(maxbutton, SIGNAL(pressed()), this, SLOT(maximise()));

    setSize(QSizeF(300, 300));

    connect(this, SIGNAL(xChanged()), m_source, SIGNAL(xChanged()));
    connect(this, SIGNAL(yChanged()), m_source, SIGNAL(yChanged()));
}

LiquidFluidityNode::~LiquidFluidityNode(){
    
}

void LiquidFluidityNode::maximise(){
    boost::shared_ptr<CauvMainWindow> in_window = m_in_window.lock();
    if(in_window){
        m_contents->setWidget(NULL);
        in_window->viewStack()->push(QString::fromStdString(m_node->nodeName()), m_view);
        m_view->setMode(f::FView::TopLevel);
        connect(m_view, SIGNAL(closeRequested()), this, SLOT(unMaximise()));
    }
}

void LiquidFluidityNode::unMaximise(){
    boost::shared_ptr<CauvMainWindow> in_window = m_in_window.lock();
    if(in_window){
        disconnect(m_view, SIGNAL(closeRequested()), this, SLOT(unMaximise()));
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
}


