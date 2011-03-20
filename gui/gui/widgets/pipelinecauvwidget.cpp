
#include "pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

using namespace cauv;

PipelineCauvWidget::PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent, boost::shared_ptr<CauvNode> node):
        CauvInterfaceElement(name, auv, node),
        m_pipeline(new pw::PipelineWidget()),
        m_observer(boost::make_shared<pw::PipelineGuiMsgObs>(m_pipeline))
{
    node->addMessageObserver(m_observer);
    m_pipeline->connect(m_pipeline, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), m_actions.get(), SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);
}

PipelineCauvWidget::~PipelineCauvWidget(){
    // m_node is a weak_ptr to break a cyclic reference
    // we need to get a strong pointer to use it
    boost::shared_ptr<CauvNode> node = m_node.lock();
    if(node) {
        node->removeMessageObserver(m_observer);
        info() << "Removed pipline message observer";
    }

    // TODO: we need to delete the pipeline here
    // but it crashes at the moment as its used by
    // multiple threads (I think this is whats happening)
    //delete m_pipeline;
}

void PipelineCauvWidget::initialise(){
    m_actions->registerCentralView(m_pipeline, name());
}
