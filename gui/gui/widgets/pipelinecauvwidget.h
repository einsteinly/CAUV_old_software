#ifndef PIPELINECAUVWIDGET_H
#define PIPELINECAUVWIDGET_H

#include <boost/shared_ptr.hpp>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

#include "../cauvinterfaceelement.h"

class PipelineCauvWidget : public pw::PipelineWidget, public CauvInterfaceElement
{
public:
    PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent, boost::shared_ptr<CauvNode> node);
    void initialise();

protected:
    boost::shared_ptr< pw::PipelineGuiMsgObs> m_observer;
};

#endif // PIPELINECAUVWIDGET_H
