#include "pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

#include <generated/messages.h>

#include <debug/cauv_debug.h>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

#include <QPushButton>
#include <QResizeEvent>
#include <QGLWidget>

using namespace cauv;


namespace cauv {
    class PipelineRefreshMessageObserver : public MessageObserver {
    public:
        PipelineRefreshMessageObserver(boost::shared_ptr<CauvNode> node) : m_node(node){
        }

        virtual void onMembershipChangedMessage(MembershipChangedMessage_ptr){
            info() << "Membership change detected, requesting pipeline listing.";
            m_node->send(boost::make_shared<PipelineDiscoveryRequestMessage>());
        }


        virtual void onPipelineDiscoveryResponseMessage(PipelineDiscoveryResponseMessage_ptr m) {
            info() << "Pipeline listing response:" << m->pipelineName();
        }

        protected:
            boost::shared_ptr<CauvNode> m_node;
    };
} // namespace cauv


PipelineCauvWidget::PipelineCauvWidget() :
        m_pipeline(new pw::PipelineWidget()),
        m_observer(boost::make_shared<pw::PipelineGuiMsgObs>(m_pipeline))
{
    m_pipeline->connect(m_pipeline, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);

    this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    this->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    this->setScene(m_pipeline);
    this->scene()->addWidget(new QPushButton("hello"));

    m_tabs.append(this);
}

PipelineCauvWidget::~PipelineCauvWidget(){
    m_node->removeMessageObserver(m_observer);
    info() << "Removed pipline message observer";

    // TODO: we need to delete the pipeline here
    // but it crashes at the moment as its used by
    // multiple threads (I think this is whats happening)
    //delete m_pipeline;
}

const QString PipelineCauvWidget::name() const{
    return QString("Pipeline");
}

const QList<QString> PipelineCauvWidget::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("pl_gui"));
    groups.push_back(QString("pipeline"));
    groups.push_back(QString("membership"));
    return groups;
}

void PipelineCauvWidget::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);

    node->addMessageObserver(m_observer);
    node->addMessageObserver(boost::make_shared<PipelineRefreshMessageObserver>(node));
}

void PipelineCauvWidget::send(boost::shared_ptr<Message> message){
    if(m_node)
        m_node->send(message);
}


void PipelineCauvWidget::resizeEvent(QResizeEvent *event)
{
    if (scene())
        scene()->setSceneRect(QRect(QPoint(0, 0), event->size()));
    QGraphicsView::resizeEvent(event);
}

Q_EXPORT_PLUGIN2(cauv_pipelineplugin, PipelineCauvWidget)
