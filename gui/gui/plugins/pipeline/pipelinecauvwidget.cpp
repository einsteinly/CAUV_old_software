#include "pipelinecauvwidget.h"
#include "pipeline/ui_pipelinecauvwidget.h"

#include <boost/make_shared.hpp>

#include <QVBoxLayout>
#include <QComboBox>

#include <debug/cauv_debug.h>
#include <generated/types/Pl_GuiGroup.h>
#include <generated/types/PipelineGroup.h>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

using namespace cauv;
using namespace cauv::gui;

PipelineListingObserver::PipelineListingObserver(boost::shared_ptr<CauvNode> node) : m_node(node), m_rate_limiter(1, 1000) {
    // rate limited to 1 per second
    qRegisterMetaType<std::string>("std::string");
}

void PipelineListingObserver::onMembershipChangedMessage(MembershipChangedMessage_ptr){
    if(m_rate_limiter.click()) {
        info() << "Membership change detected, requesting pipeline listing.";
        m_node->send(boost::make_shared<PipelineDiscoveryRequestMessage>());
        Q_EMIT searchStarted();
    } else {
        debug(3) << "Membership change detected but not requesting discovery due to rate limiting";
    }
}

void PipelineListingObserver::onPipelineDiscoveryRequestMessage(PipelineDiscoveryRequestMessage_ptr){
    // listen for discovery messages from others to include them in the
    // the rate limiting
    // doesn't matter if this fails as we're not actually adding to the rate here
    m_rate_limiter.click();
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
    m_pipeline->connect(m_pipeline, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)), this, SLOT(send(boost::shared_ptr<const Message>)));

    ui->pipelines->hide(); // hide until its populated
    ui->layout->addWidget(m_pipeline);

    ui->pipelines->connect(ui->pipelines, SIGNAL(currentIndexChanged(const QString&)), m_pipeline, SLOT(setPipelineName(const QString&)));

    m_tabs.append(this);
}

PipelineCauvWidget::~PipelineCauvWidget(){
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

    //listingObserver->connect(listingObserver.get(), SIGNAL(searchStarted()), this, SLOT(clearPipelines()));
    listingObserver->connect(listingObserver.get(), SIGNAL(pipelineDiscovered(std::string)), this, SLOT(addPipeline(std::string)));
}

void PipelineCauvWidget::addPipeline(std::string name){

    // check if its an unknown pipeline to us
    if(ui->pipelines->findText(QString::fromStdString(name)) < 0) {

        ui->pipelines->blockSignals(true);

        ui->pipelines->addItem(QString::fromStdString(name));

        if(ui->pipelines->count() > 1)
            ui->pipelines->show();

        // try to sync with the currently displayed pipeline
        ui->pipelines->setCurrentIndex(ui->pipelines->findText(QString::fromStdString(m_pipeline->pipelineName())));

        ui->pipelines->blockSignals(false);
    }
}

void PipelineCauvWidget::clearPipelines(){
    ui->pipelines->blockSignals(true);
    ui->pipelines->clear();
    ui->pipelines->hide();
    ui->pipelines->blockSignals(false);
}

void PipelineCauvWidget::send(boost::shared_ptr<const Message> message){
    info() << message;
    if(m_node)
        m_node->send(message);
}

Q_EXPORT_PLUGIN2(cauv_pipelineplugin, PipelineCauvWidget)
