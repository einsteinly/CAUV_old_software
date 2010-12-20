#include "pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

using namespace cauv;

PipelineCauvWidget::PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent):
        pw::PipelineWidget(parent),
        CauvInterfaceElement(name, auv),
        m_observer(boost::make_shared<pw::PipelineGuiMsgObs>(this))
{
    connect(this, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), m_actions.get(), SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);
}


void PipelineCauvWidget::initialise(){
    m_actions->registerCentralView(this, name());
}
