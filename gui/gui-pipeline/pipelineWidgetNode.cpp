#include "pipelineWidgetNode.h"
#include "pipelineMessageObserver.h"

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <generated/types/DebugGroup.h>
#include <generated/types/Pl_GuiGroup.h>
#include <generated/types/PipelineGroup.h>

cauv::pw::PipelineGuiCauvNode::PipelineGuiCauvNode(PipelineWidget *p)
    : CauvNode("pipe-gui"), m_widget(p){
    p->connect(p, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);
    debug() << "PGCN constructed";
}

void cauv::pw::PipelineGuiCauvNode::onRun(){
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

int cauv::pw::PipelineGuiCauvNode::send(boost::shared_ptr<Message> message){
    debug() << "Sending message" << *message << std::endl;
    return CauvNode::send(message);
}

void cauv::pw::spawnPGCN(PipelineWidget *p, int argc, char** argv){
    boost::shared_ptr<PipelineGuiCauvNode> pgcn =
        boost::make_shared<PipelineGuiCauvNode>(p);

    //p->setCauvNode(pgcn); - removed by Andy Pritchard 11/12/2010
    // no longer needed as its decoupled now

    pgcn->parseOptions(argc, argv);
    pgcn->run();
    warning() << __func__ << "run() finished";
}

