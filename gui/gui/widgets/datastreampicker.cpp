#include "datastreampicker.h"
#include "../datastreamdragging.h"
#include "../utils/treemodel.h"

#include "ui_datastreampicker.h"

#include <QModelIndexList>

using namespace cauv;



DataStreamTreeItemBase::DataStreamTreeItemBase(boost::shared_ptr<DataStreamBase> stream, QTreeWidgetItem * parent):
        QTreeWidgetItem(parent), m_stream(stream){
    if(stream->isMutable()) {
        setTextColor(1, QColor::fromRgb(52, 138, 52));
    }
}

boost::shared_ptr<DataStreamBase> DataStreamTreeItemBase::getDataStreamBase(){
    return m_stream;
}




template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    this->setText(1, QString::fromStdString(stream.str()));
}
// another for int8_t for much the same reason but with lexical cast this time
template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant &value){
    return boost::lexical_cast<int>(value.toString().toStdString());
}

// also need some for out types as lexical cast doesn't knwo what to do
template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& ){
    // TODO: implement, should recognise something like (1.0, 2.0, 3.0)
    throw new boost::bad_lexical_cast;
    return floatYPR();
}

template<> sonar_params_t DataStreamTreeItem<sonar_params_t>::qVariantToValue(QVariant& ){
    // TODO: implement it
    throw new boost::bad_lexical_cast;
    return sonar_params_t();
}
template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& ){
    throw new boost::bad_lexical_cast;
    return Image();
}




boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > DataStreamList::getDataStreams() const {
    boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > streams = boost::make_shared<std::vector<boost::shared_ptr<DataStreamBase> > >();

    QModelIndexList items = this->selectedIndexes();
    QModelIndexList::iterator i;
    for (i = items.begin(); i != items.end(); ++i){
        QModelIndex index = (*i);
        if(index.column() == 0) {
            QTreeWidgetItem *item = static_cast<QTreeWidgetItem*>(index.internalPointer());
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

    ui->dataStreams->setRootIsDecorated( true );
    ui->dataStreams->setDragEnabled(true);
    ui->dataStreams->setDropIndicatorShown(true);
    ui->dataStreams->setAcceptDrops(false);


    // set up the categories
    QTreeWidgetItem *motors = new QTreeWidgetItem(ui->dataStreams);
    motors->setText(0, "Motors");
    motors->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    motors->setExpanded(true);

    new DataStreamTreeItem<int8_t>(auv->motors.prop, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.hbow, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.vbow, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.hstern, motors);
    new DataStreamTreeItem<int8_t>(auv->motors.vstern, motors);

    QTreeWidgetItem *autopilots = new QTreeWidgetItem(ui->dataStreams);
    autopilots->setText(0, "Autopilots");
    autopilots->setFlags(autopilots->flags() ^ Qt::ItemIsSelectable);
    autopilots->setExpanded(true);

    DataStreamTreeItem<float> *bearing = new DataStreamTreeItem<float>(auv->autopilots.bearing, autopilots);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kP, "kP", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kI, "kI", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->kD, "kD", bearing);
    new DataStreamTreeItem<float>(auv->autopilots.bearing->scale, "scale", bearing);

    DataStreamTreeItem<float> *depth = new DataStreamTreeItem<float>(auv->autopilots.depth, autopilots);
    new DataStreamTreeItem<float>(auv->autopilots.depth->kP, "kP", depth);
    new DataStreamTreeItem<float>(auv->autopilots.depth->kI, "kI", depth);
    new DataStreamTreeItem<float>(auv->autopilots.depth->kD, "kD", depth);
    new DataStreamTreeItem<float>(auv->autopilots.depth->scale, "scale", depth);

    DataStreamTreeItem<float> *pitch = new DataStreamTreeItem<float>(auv->autopilots.pitch, autopilots);
    new DataStreamTreeItem<float>(auv->autopilots.pitch->kP, "kP", pitch);
    new DataStreamTreeItem<float>(auv->autopilots.pitch->kI, "kI", pitch);
    new DataStreamTreeItem<float>(auv->autopilots.pitch->kD, "kD", pitch);
    new DataStreamTreeItem<float>(auv->autopilots.pitch->scale, "scale", pitch);

    QTreeWidgetItem *cameras = new QTreeWidgetItem(ui->dataStreams);
    cameras->setText(0, "Imaging");
    cameras->setFlags(cameras->flags() ^ Qt::ItemIsSelectable);
    cameras->setExpanded(true);

    new DataStreamTreeItem<Image>(auv->cameras.forward, cameras);
    new DataStreamTreeItem<Image>(auv->cameras.down, cameras);
    DataStreamTreeItem<Image> * sonar = new DataStreamTreeItem<Image>(auv->cameras.sonar, cameras);
    new DataStreamTreeItem<sonar_params_t>(auv->cameras.sonar->params, "Params", sonar);

    QTreeWidgetItem *sensors = new QTreeWidgetItem(ui->dataStreams);
    sensors->setText(0, "Sensors");
    sensors->setFlags(sensors->flags() ^ Qt::ItemIsSelectable);
    sensors->setExpanded(true);

    new DataStreamTreeItem<uint16_t>(auv->sensors.pressure_fore, sensors);
    new DataStreamTreeItem<uint16_t>(auv->sensors.pressure_aft, sensors);
    new DataStreamTreeItem<float>(auv->sensors.depth, sensors);
    DataStreamTreeItem<floatYPR> * orientation = new DataStreamTreeItem<floatYPR>(auv->sensors.orientation, sensors);
    new DataStreamTreeItem<float>(auv->sensors.orientation_split->yaw, orientation);
    new DataStreamTreeItem<float>(auv->sensors.orientation_split->pitch, orientation);
    new DataStreamTreeItem<float>(auv->sensors.orientation_split->roll, orientation);
}

DataStreamPicker::~DataStreamPicker(){
    delete ui;
}

void DataStreamPicker::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}
