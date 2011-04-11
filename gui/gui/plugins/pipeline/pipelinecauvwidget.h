#ifndef PIPELINECAUVWIDGET_H
#define PIPELINECAUVWIDGET_H

#include <boost/shared_ptr.hpp>

#include <QObject>

#include <gui/core/cauvbasicplugin.h>

#include <generated/messages_fwd.h>

namespace cauv {

    namespace pw {
        class PipelineWidget;
        class PipelineGuiMsgObs;
    }

    class PipelineCauvWidget : public QObject, public CauvBasicPlugin
    {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:
        PipelineCauvWidget();
        virtual ~PipelineCauvWidget();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    protected Q_SLOTS:
        void send(boost::shared_ptr<Message> message);

    protected:
        pw::PipelineWidget * m_pipeline;
        boost::shared_ptr< pw::PipelineGuiMsgObs> m_observer;
    };

} // namespace cauv

#endif // PIPELINECAUVWIDGET_H
