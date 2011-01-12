#include "datastreampicker.h"
#include "../datastreamdragging.h"
#include "../utils/treemodel.h"

#include "ui_datastreampicker.h"

#include <QModelIndexList>

using namespace cauv;


template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    setValue(stream.str());
}

boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > DataStreamList::getDataStreams() const {
    boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > streams = boost::make_shared<std::vector<boost::shared_ptr<DataStreamBase> > >();

    QModelIndexList items = this->selectedIndexes();
    QModelIndexList::iterator i;
    for (i = items.begin(); i != items.end(); ++i){
        QModelIndex index = (*i);
        if(index.column() == 0) {
            TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
            DataStreamTreeItemBase * dsItem = dynamic_cast<DataStreamTreeItemBase*>(item);
            if(dsItem)
                streams->push_back(dsItem->getDataStreamBase());
        }
    }
    return streams;
}


DataStreamPicker::DataStreamPicker(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv, node),
    ui(new Ui::DataStreamPicker())
{
    ui->setupUi(this);

    ui->dataStreamsView->setRootIsDecorated( true );
    ui->dataStreamsView->setDragEnabled(true);
    ui->dataStreamsView->setDropIndicatorShown(true);
    ui->dataStreamsView->setAcceptDrops(false);

    TreeItem * root = new TreeItem();
    root->appendData(QVariant(QString::fromStdString("Stream")));
    root->appendData(QVariant(QString::fromStdString("Value")));
    ui->dataStreamsView->setModel(new TreeModel(root));

    // set up the categories

    TreeItem *motors = new TreeItem(root);
    motors->appendData(QVariant(QString::fromStdString("Motors")));

    new DataStreamTreeItem<int8_t>(auv->motors.prop, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.hbow, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.vbow, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.hstern, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.vstern, motors);



    TreeItem *autopilots = new TreeItem(root);
    autopilots->appendData(QVariant(QString::fromStdString("Autopilots")));

    TreeItem *bearing = new DataStreamTreeItem<float>(auv->autopilots.bearing, autopilots);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kP, "kP", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kI, "kI", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kD, "kD", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->scale, "scale", bearing);
/*
    QTreeWidgetItem *depth = new QTreeWidgetItem(autopilots);
    depth->setText(0, "Depth");
    depth->setFlags(depth->flags() ^ Qt::ItemIsSelectable);
    (new DataStreamTreeItem<float>(depth, auv->autopilots.depth))->setText(0, "Target");
    DataStreamTreeItem<autopilot_params_t>* depthParams = new DataStreamTreeItem<autopilot_params_t>(depth, auv->autopilots.depth->params);
    depthParams->setText(0, "Params");
    (new DataStreamTreeItem<float>(depthParams, auv->autopilots.depth->kP))->setText(0, "kP");
    (new DataStreamTreeItem<float>(depthParams, auv->autopilots.depth->kI))->setText(0, "kI");
    (new DataStreamTreeItem<float>(depthParams, auv->autopilots.depth->kD))->setText(0, "kD");
    (new DataStreamTreeItem<float>(depthParams, auv->autopilots.depth->scale))->setText(0, "Scale");

    QTreeWidgetItem *pitch = new QTreeWidgetItem(autopilots);
    pitch->setText(0, "Pitch");
    pitch->setFlags(pitch->flags() ^ Qt::ItemIsSelectable);
    (new DataStreamTreeItem<float>(pitch, auv->autopilots.pitch))->setText(0, "Target");
    DataStreamTreeItem<autopilot_params_t>* pitchParams = new DataStreamTreeItem<autopilot_params_t>(pitch, auv->autopilots.pitch->params);
    pitchParams->setText(0, "Params");
    (new DataStreamTreeItem<float>(pitchParams, auv->autopilots.pitch->kP))->setText(0, "kP");
    (new DataStreamTreeItem<float>(pitchParams, auv->autopilots.pitch->kI))->setText(0, "kI");
    (new DataStreamTreeItem<float>(pitchParams, auv->autopilots.pitch->kD))->setText(0, "kD");
    (new DataStreamTreeItem<float>(pitchParams, auv->autopilots.pitch->scale))->setText(0, "Scale");


    QTreeWidgetItem *cameras = new QTreeWidgetItem(dataStreams);
    cameras->setText(0, "Imaging");
    cameras->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    cameras->setExpanded(true);

    new DataStreamTreeItem<Image>(cameras, auv->cameras.forward);
    new DataStreamTreeItem<Image>(cameras, auv->cameras.down);
    DataStreamTreeItem<Image> * sonar = new DataStreamTreeItem<Image>(cameras, auv->cameras.sonar);
    (new DataStreamTreeItem<sonar_params_t>(sonar, auv->cameras.sonar->params))->setText(0, "Params");

    QTreeWidgetItem *sensors = new QTreeWidgetItem(dataStreams);
    sensors->setText(0, "Sensors");
    sensors->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    sensors->setExpanded(true);

    new DataStreamTreeItem<uint16_t>(sensors, auv->sensors.pressure_fore);
    new DataStreamTreeItem<uint16_t>(sensors, auv->sensors.pressure_aft);
    new DataStreamTreeItem<float>(sensors, auv->sensors.depth);
    DataStreamTreeItem<floatYPR> * orientation = new DataStreamTreeItem<floatYPR>(sensors, auv->sensors.orientation);
    new DataStreamTreeItem<float>(orientation, auv->sensors.orientation_split->yaw);
    new DataStreamTreeItem<float>(orientation, auv->sensors.orientation_split->pitch);
    new DataStreamTreeItem<float>(orientation, auv->sensors.orientation_split->roll); */
}

DataStreamPicker::~DataStreamPicker(){
    delete ui;
}

void DataStreamPicker::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}
