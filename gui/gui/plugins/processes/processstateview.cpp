#include "processstateview.h"
#include "processes/ui_processstateview.h"

#include <model/auv_model.h>

using namespace cauv;

ProcessStateView::ProcessStateView() :
        ui(new Ui::ProcessStateView)
{
    ui->setupUi(this);

    connect(this, SIGNAL(processStateUpdated(std::string,float,float,float, std::string)), this, SLOT(onProcessStateUpdate(std::string,float,float,float, std::string)));

    QStringList list;
    list << "Process" << "CPU" << "Mem" << "Threads" << "Status";
    ui->tableWidget->setHorizontalHeaderLabels(list);

    m_tabs.append(this);
}

ProcessStateView::~ProcessStateView()
{
    delete ui;
}

void ProcessStateView::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);
    auv->computer_state.processes->onUpdate.connect(boost::bind(&ProcessStateView::onProcessStateUpdate, this, _1));
}

void ProcessStateView::onProcessStateUpdate(const std::string process, const float cpu, const float mem, const float threads, std::string status){

    int row = m_processes.size();
    try {
        row = m_processes.at(process);
    } catch (...) {
        ui->tableWidget->setRowCount(row+1);
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem("Process"));
        ui->tableWidget->setItem(row, 1, new QTableWidgetItem("CPU"));
        ui->tableWidget->setItem(row, 2, new QTableWidgetItem("Memory"));
        ui->tableWidget->setItem(row, 3, new QTableWidgetItem("Threads"));
        ui->tableWidget->setItem(row, 4, new QTableWidgetItem("Status"));
        m_processes[process] = row;
    }

    QTableWidgetItem * nameWidget = ui->tableWidget->item(row, 0);
    QTableWidgetItem * cpuWidget = ui->tableWidget->item(row, 1);
    QTableWidgetItem * memWidget = ui->tableWidget->item(row, 2);
    QTableWidgetItem * threadsWidget = ui->tableWidget->item(row, 3);
    QTableWidgetItem * statusWidget = ui->tableWidget->item(row, 4);

    if(nameWidget)
        nameWidget->setText(QString::fromStdString(process));
    else error() << "nameWidget not set somehow";
    if(cpuWidget)
        cpuWidget->setText(QString::number(cpu));
    else error() << "cpuWidget not set somehow";
    if(memWidget)
        memWidget->setText(QString::number(mem));
    else error() << "memWidget not set somehow";
    if(threadsWidget)
        threadsWidget->setText(QString::number(threads));
    else error() << "threadsWidget not set somehow";
    if(statusWidget)
        statusWidget->setText(QString::fromStdString(status));
    else error() << "statusWidget not set somehow";
}

void ProcessStateView::onProcessStateUpdate(const ProcessState &state){
    onProcessStateUpdate(state.process(), state.cpu(), state.mem(), state.threads(), state.status());
}

const QString ProcessStateView::name() const{
    return QString("Processes");
}

const QList<QString> ProcessStateView::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("telemetry"));
    return groups;
}

Q_EXPORT_PLUGIN2(cauv_processesplugin, ProcessStateView)
