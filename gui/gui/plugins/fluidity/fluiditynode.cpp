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


LiquidFluidityNode::LiquidFluidityNode(boost::shared_ptr<FluidityNode> node, QGraphicsItem *parent)
    : liquid::LiquidNode(AI_Node_Style(), parent),
      ManagedNode(this, node),
      m_node(node),
      m_contents(NULL),
      m_source(new liquid::ArcSource(this, new liquid::Arc(Image_Arc_Style()))){

    setResizable(true);

    header()->setTitle(QString::fromStdString(m_node->nodeName()));
    header()->setInfo(QString::fromStdString(m_node->nodePath()));

    m_source->setParentItem(this);
    m_source->setZValue(10);
    
    boost::shared_ptr<CauvNode> cauv_node = FluidityPlugin::theCauvNode().lock();
    if(cauv_node){
        f::FView* view = new f::FView(cauv_node, m_node->nodeName());
        view->setMode(f::FView::Internal);
        m_contents = new liquid::ProxyWidget(this);
        m_contents->setWidget(view);

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

    setSize(QSizeF(300, 300));

    connect(this, SIGNAL(xChanged()), m_source, SIGNAL(xChanged()));
    connect(this, SIGNAL(yChanged()), m_source, SIGNAL(yChanged()));
}

LiquidFluidityNode::~LiquidFluidityNode(){
    
}

