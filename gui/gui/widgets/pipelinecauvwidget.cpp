
#include "pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

using namespace cauv;

PipelineCauvWidget::PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent, boost::shared_ptr<CauvNode> node):
        CauvInterfaceElement(name, auv, node),
        m_pipeline(new pw::PipelineWidget(parent)),
        m_observer(boost::make_shared<pw::PipelineGuiMsgObs>(m_pipeline))
{
    node->addMessageObserver(m_observer);
    m_pipeline->connect(m_pipeline, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), m_actions.get(), SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);
}

PipelineCauvWidget::~PipelineCauvWidget(){
    delete m_pipeline;
}

void PipelineCauvWidget::initialise(){
    m_actions->registerCentralView(m_pipeline, name());
}
