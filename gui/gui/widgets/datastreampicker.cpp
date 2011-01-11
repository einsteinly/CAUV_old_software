#include "datastreampicker.h"
#include "../datastreamdragging.h"

#include <QModelIndexList>

using namespace cauv;

boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > DataStreamList::getDataStreams() const {
    boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > streams = boost::make_shared<std::vector<boost::shared_ptr<DataStreamBase> > >();

    QModelIndexList items = this->selectedIndexes();
    QModelIndexList::iterator i;
    for (i = items.begin(); i != items.end(); ++i){
        //(*i);
    }
    return streams;
}


DataStreamPicker::DataStreamPicker(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv, node)
{
    setupUi(this);

    dataStreams->setRootIsDecorated( true );

    dataStreams->setDragEnabled(true);
    dataStreams->setDropIndicatorShown(true);
    dataStreams->setAcceptDrops(false);


    // set up the categories
    QTreeWidgetItem *motors = new QTreeWidgetItem(dataStreams);
    motors->setText(0, "Motors");
    motors->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    motors->setExpanded(true);

    new DataStreamTreeItem<int8_t>(motors, auv->motors.prop);
    new DataStreamTreeItem<int8_t>(motors, auv->motors.hbow);
    new DataStreamTreeItem<int8_t>(motors, auv->motors.vbow);
    new DataStreamTreeItem<int8_t>(motors, auv->motors.hstern);
    new DataStreamTreeItem<int8_t>(motors, auv->motors.vstern);

    QTreeWidgetItem *autopilots = new QTreeWidgetItem(dataStreams);
    autopilots->setText(0, "Autopilots");
    autopilots->setFlags(autopilots->flags() ^ Qt::ItemIsSelectable);
    autopilots->setExpanded(true);

    QTreeWidgetItem *bearing = new QTreeWidgetItem(autopilots);
    bearing->setText(0, "Bearing");
    bearing->setFlags(bearing->flags() ^ Qt::ItemIsSelectable);
    (new DataStreamTreeItem<float>(bearing, auv->autopilots.bearing))->setText(0, "Target");
    DataStreamTreeItem<autopilot_params_t>* bearingParams = new DataStreamTreeItem<autopilot_params_t>(bearing, auv->autopilots.bearing->params);
    bearingParams->setText(0, "Params");
    (new DataStreamTreeItem<float>(bearingParams, auv->autopilots.bearing->kP))->setText(0, "kP");
    (new DataStreamTreeItem<float>(bearingParams, auv->autopilots.bearing->kI))->setText(0, "kI");
    (new DataStreamTreeItem<float>(bearingParams, auv->autopilots.bearing->kD))->setText(0, "kD");
    (new DataStreamTreeItem<float>(bearingParams, auv->autopilots.bearing->scale))->setText(0, "Scale");

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
    new DataStreamTreeItem<float>(orientation, auv->sensors.orientation_split->roll);
}

void DataStreamPicker::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}
