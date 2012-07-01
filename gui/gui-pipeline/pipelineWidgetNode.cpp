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

#include "pipelineWidgetNode.h"
#include "pipelineMessageObserver.h"

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <generated/types/DebugGroup.h>
#include <generated/types/Pl_GuiGroup.h>
#include <generated/types/PipelineGroup.h>

cauv::gui::pw::PipelineGuiCauvNode::PipelineGuiCauvNode(PipelineWidget *p)
    : CauvNode("pipe-gui"), m_widget(p){

    p->connect(p, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);
    debug() << "PGCN constructed";
}

void cauv::gui::pw::PipelineGuiCauvNode::onRun(){
    debug() << "PGCN::onRun()";
    joinGroup("pl_gui");

    addMessageObserver(
        boost::make_shared<PipelineGuiMsgObs>(m_widget)
    );
    addMessageObserver(
        boost::make_shared<DBGLevelObserver>()
    );
    #if defined(USE_DEBUG_MESSAGE_OBSERVERS)
    addMessageObserver(
        boost::make_shared<DebugMessageObserver>()
    );
    #endif

    // get the initial pipeline state:
    send(boost::make_shared<GraphRequestMessage>(m_widget->pipelineName()));
}

int cauv::gui::pw::PipelineGuiCauvNode::send(boost::shared_ptr<Message> message){
    std::cout << "Sending message" << *message << std::endl;
    return CauvNode::send(message);
}

void cauv::gui::pw::spawnPGCN(PipelineWidget *p, int argc, char** argv){
    boost::shared_ptr<PipelineGuiCauvNode> pgcn =
        boost::make_shared<PipelineGuiCauvNode>(p);

    //p->setCauvNode(pgcn); - removed by Andy Pritchard 11/12/2010
    // no longer needed as its decoupled now

    pgcn->parseOptions(argc, argv);
    pgcn->run();
    warning() << __func__ << "run() finished";
}

