#include "datastreampicker.h"
#include "../datastreamdragging.h"
#include "../utils/treemodel.h"

#include "ui_datastreampicker.h"

#include <QModelIndexList>
#include <opencv/cv.h>

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

template<> void DataStreamTreeItem<Image>::onChange(const Image value){

    try {
        cv::Mat mat_rgb;
        cv::cvtColor(value.cvMat(), mat_rgb, CV_BGR2RGB);

        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                               mat_rgb.rows, QImage::Format_RGB888);

        this->setIcon(1, QIcon(QPixmap::fromImage(qImage)));

    } catch (cv::Exception ex){
        error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__ << " " << ex.msg;
    }
}

template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant &value){
    return boost::lexical_cast<int>(value.toString().toStdString());
}

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




DataStreamList::DataStreamList(QWidget * parent) : QTreeWidget(parent){
    // Because QTreeWidgetItem can't let only one cell in a row be editable we have to hack it
    // around a bit. We set the item to be editable when it's double clicked in the editable
    // cell. This is then removed once the focus is lost. It works but its not a very elegant solution
    // TODO: Better would be to use a full QAbstractItemModel implementation but this is quite a
    // big job as QAbstractItemModels are complicated.
    connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), this, SLOT(editStarted(QTreeWidgetItem*,int)));
    connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*,QTreeWidgetItem*)), this, SLOT(editEnded(QTreeWidgetItem*,QTreeWidgetItem*)));
    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(itemEdited(QTreeWidgetItem*,int)));
}


void DataStreamList::editStarted(QTreeWidgetItem* item, int column){
    if(column == 1){
        cauv::DataStreamTreeItemBase * dsItem = dynamic_cast<cauv::DataStreamTreeItemBase*>(item);
        if(dsItem && dsItem->getDataStreamBase()->isMutable()){
            item->setFlags(item->flags() | Qt::ItemIsEditable);
        }
    }
}

void DataStreamList::itemEdited(QTreeWidgetItem* item, int column){
    cauv::DataStreamTreeItemBase * dsItem = dynamic_cast<cauv::DataStreamTreeItemBase*>(item);

    // 1 is the editable cell that stores the value
    if(column == 1 && dsItem && dsItem->getDataStreamBase()->isMutable()){
        QVariant v = item->data(column, Qt::DisplayRole);
        if(!v.toString().isEmpty()) {
            // if the item is marked as editable then the change came from a user interaction
            // not a stream update
            // TODO: find a better way of doing this. it's a bit hacky.
            if(item->flags() & Qt::ItemIsEditable) {
                if(dsItem->updateStream(v)) item->setBackground(1, QBrush());
                else  {
                    QBrush b(Qt::DiagCrossPattern);
                    b.setColor(QColor::fromRgb(224, 128, 128));
                    item->setBackground(1, b);
                }
            }
        }
    }
}

void DataStreamList::editEnded(QTreeWidgetItem* , QTreeWidgetItem* previous){
    if(previous) // this method resets the editable flag if the focus is lost
        previous->setFlags(previous->flags() & (~Qt::ItemIsEditable));
}

