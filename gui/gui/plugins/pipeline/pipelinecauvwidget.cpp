#include "pipelinecauvwidget.h"
#include "pipeline/ui_pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

#include <QVBoxLayout>
#include <QComboBox>

#include <generated/messages.h>

#include <debug/cauv_debug.h>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

using namespace cauv;


PipelineListingObserver::PipelineListingObserver(boost::shared_ptr<CauvNode> node) : m_node(node) {
    qRegisterMetaType<std::string>("std::string");
}

void PipelineListingObserver::onMembershipChangedMessage(MembershipChangedMessage_ptr){
    info() << "Membership change detected, requesting pipeline listing.";
    m_node->send(boost::make_shared<PipelineDiscoveryRequestMessage>());
    Q_EMIT searchStarted();
}

void PipelineListingObserver::onPipelineDiscoveryResponseMessage(PipelineDiscoveryResponseMessage_ptr m) {
    info() << "Pipeline listing response:" << m->pipelineName();
    Q_EMIT pipelineDiscovered(m->pipelineName());
}


PipelineCauvWidget::PipelineCauvWidget() :
        m_pipeline(new pw::PipelineWidget()),
        m_observer(boost::make_shared<pw::PipelineGuiMsgObs>(m_pipeline)),
        ui(new Ui::PipelineCauvWidget())
{
    ui->setupUi(this);
    m_pipeline->connect(m_pipeline, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);

    ui->pipelines->hide(); // hide until its populated
    ui->layout->addWidget(m_pipeline);

    ui->pipelines->connect(ui->pipelines, SIGNAL(currentIndexChanged(const QString&)), m_pipeline, SLOT(setPipelineName(const QString&)));

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
    boost::shared_ptr<PipelineListingObserver> listingObserver = boost::make_shared<PipelineListingObserver>(node);
    node->addMessageObserver(listingObserver);

    listingObserver->connect(listingObserver.get(), SIGNAL(searchStarted()), ui->pipelines, SLOT(clear()));
    listingObserver->connect(listingObserver.get(), SIGNAL(searchStarted()), ui->pipelines, SLOT(hide()));
    listingObserver->connect(listingObserver.get(), SIGNAL(pipelineDiscovered(std::string)), this, SLOT(addPipeline(std::string)));
}

void PipelineCauvWidget::addPipeline(std::string name){

    // check if its an unknown pipeline to us
    if(ui->pipelines->findText(QString::fromStdString(name)) < 0) {

        ui->pipelines->addItem(QString::fromStdString(name));

        if(ui->pipelines->count() > 1)
            ui->pipelines->show();
    }
}

void PipelineCauvWidget::send(boost::shared_ptr<Message> message){
    if(m_node)
        m_node->send(message);
}

Q_EXPORT_PLUGIN2(cauv_pipelineplugin, PipelineCauvWidget)
