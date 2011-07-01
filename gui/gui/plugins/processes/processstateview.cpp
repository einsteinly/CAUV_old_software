#include "processstateview.h"
#include "processes/ui_processstateview.h"

#include <debug/cauv_debug.h>
#include <model/auv_model.h>

#include <QModelIndexList>
#include <QTableWidgetItem>

using namespace cauv;

namespace cauv {

    class DataStreamTableItem : public QTableWidgetItem {

    public:
        DataStreamTableItem(boost::shared_ptr<DataStreamBase> dataStream, const QString &text = "", int type = Type) :
                QTableWidgetItem(text, type), m_stream(dataStream)
        {
        }

        boost::shared_ptr<DataStreamBase> getDataStream() const {
            return m_stream;
        }

    protected:
        boost::shared_ptr<DataStreamBase> m_stream;

    };

} // namespace cauv


ProcessStateView::ProcessStateView() :
        ui(new Ui::ProcessStateView)
{
    ui->setupUi(this);

    connect(this, SIGNAL(processStateUpdated(std::string,float,float,float, std::string)), this, SLOT(onProcessStateUpdate(std::string,float,float,float, std::string)));

    QStringList list;
    list << "Process" << "CPU" << "Mem" << "Threads" << "Status";
    ui->tableWidget->setHorizontalHeaderLabels(list);

    //m_tabs.append(this);
    m_docks[this] = Qt::RightDockWidgetArea;
}

ProcessStateView::~ProcessStateView()
{
    delete ui;
}

void ProcessStateView::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);
    auv->computer_state.new_process_stream->onUpdate.connect(boost::bind(&ProcessStateView::onNewProcess, this, _1));
}


boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > DraggableTableWidget::getDataStreams() {
    boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > streams = boost::make_shared<std::vector<boost::shared_ptr<DataStreamBase> > >();

    QList<QTableWidgetItem *> items = selectedItems();

    foreach(QTableWidgetItem * item, items){
        cauv::DataStreamTableItem * dsItem = dynamic_cast<cauv::DataStreamTableItem *>(item);
        if(dsItem){
            streams->push_back(dsItem->getDataStream());
        }
    }

    return streams;
}

void ProcessStateView::onNewProcess(boost::shared_ptr<DataStream<ProcessState> > stream) {
    int row = m_processes.size();
    try {
        row = m_processes.at(stream->getName());
        return; // not acutally new
    } catch (...) {
        // new splitter required

        boost::shared_ptr<DataStreamSplitter<ProcessState> > splitter = boost::make_shared<DataStreamSplitter<ProcessState> >(stream);
        stream->onUpdate.connect(boost::bind(&ProcessStateView::onProcessStateUpdate, this, _1));
        m_splitters.push_back(splitter);

        ui->tableWidget->setRowCount(row+1);
        ui->tableWidget->setItem(row, 0, new DataStreamTableItem(splitter->process));
        ui->tableWidget->setItem(row, 1, new DataStreamTableItem(splitter->cpu));
        ui->tableWidget->setItem(row, 2, new DataStreamTableItem(splitter->mem));
        ui->tableWidget->setItem(row, 3, new DataStreamTableItem(splitter->threads));
        ui->tableWidget->setItem(row, 4, new DataStreamTableItem(splitter->status));
        m_processes[stream->getName()] = row;
    }
}

void ProcessStateView::onProcessStateUpdate(const ProcessState &state){
    onProcessStateUpdate(state.process(), state.cpu(), state.mem(), state.threads(), state.status());
}

void ProcessStateView::onProcessStateUpdate(const std::string process, const float cpu, const float mem, const float threads, std::string status){

    try {
        int row = m_processes.at(process);

        QTableWidgetItem * nameWidget = ui->tableWidget->item(row, 0);
        QTableWidgetItem * cpuWidget = ui->tableWidget->item(row, 1);
        QTableWidgetItem * memWidget = ui->tableWidget->item(row, 2);
        QTableWidgetItem * threadsWidget = ui->tableWidget->item(row, 3);
        QTableWidgetItem * statusWidget = ui->tableWidget->item(row, 4);

        if(nameWidget)
            nameWidget->setText(QString::fromStdString(process));
        else error() << "nameWidget not set somehow";
        if(cpuWidget){
            if(cpu != -1)
                cpuWidget->setText(QString::number(cpu));
            else cpuWidget->setText(QString("n/a"));
        }
        else error() << "cpuWidget not set somehow";
        if(memWidget){
            if(mem != -1)
                memWidget->setText(QString::number(mem));
            else memWidget->setText(QString("n/a"));
        }
        else error() << "memWidget not set somehow";
        if(threadsWidget)
            threadsWidget->setText(QString::number(threads));
        else error() << "threadsWidget not set somehow";
        if(statusWidget)
            statusWidget->setText(QString::fromStdString(status));
        else error() << "statusWidget not set somehow";

    } catch (...) {
    }
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
