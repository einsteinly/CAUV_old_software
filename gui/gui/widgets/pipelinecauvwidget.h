#ifndef PIPELINECAUVWIDGET_H
#define PIPELINECAUVWIDGET_H

#include <boost/shared_ptr.hpp>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

#include "ui_pipelinecauvwidget.h"
#include "../cauvinterfaceelement.h"

namespace cauv{

class PipelineCauvWidget : public Ui::PipelineCauvWidget, public pw::PipelineWidget, public CauvInterfaceElement
{
public:
    PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent);
    void initialise();

protected:
    boost::shared_ptr< pw::PipelineGuiMsgObs> m_observer;
};

} // namespace cauv

#endif // PIPELINECAUVWIDGET_H
