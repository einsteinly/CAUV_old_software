#ifndef PIPELINECAUVWIDGET_H
#define PIPELINECAUVWIDGET_H

#include <boost/shared_ptr.hpp>

#include "gui/cauvinterfaceelement.h"

namespace cauv {

    namespace pw {
        class PipelineWidget;
        class PipelineGuiMsgObs;
    }

    class PipelineCauvWidget : public CauvInterfaceElement
    {
    public:
        PipelineCauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent, boost::shared_ptr<CauvNode> node);
        virtual ~PipelineCauvWidget();
        void initialise();

    protected:
        pw::PipelineWidget * m_pipeline;
        boost::shared_ptr< pw::PipelineGuiMsgObs> m_observer;
    };

} // namespace cauv

#endif // PIPELINECAUVWIDGET_H
