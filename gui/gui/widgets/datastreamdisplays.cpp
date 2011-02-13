
#include "datastreamdisplays.h"
#include "ui_datastreamdisplays.h"

#include "datastreamdragging.h"
#include "videoscreen.h"
#include "graphs.h"

#include <QMdiSubWindow>
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
        if (value.cvMat().empty())
            return;

        cv::Mat mat_rgb;
        cv::cvtColor(value.cvMat(), mat_rgb, CV_BGR2RGB);

        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                               mat_rgb.rows, QImage::Format_RGB888);

        //this->setIcon(1, QIcon(QPixmap::fromImage(qImage)));

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
    // motors
    QTreeWidgetItem *motors = new QTreeWidgetItem(ui->dataStreams);
    motors->setText(0, "Motors");
    motors->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    motors->setExpanded(true);

    foreach(AUV::motor_map::value_type i, auv->motors) {
        new DataStreamTreeItem<int8_t>(i.second, motors);
    }

    // autopilots
    QTreeWidgetItem *autopilots = new QTreeWidgetItem(ui->dataStreams);
    autopilots->setText(0, "Autopilots");
    autopilots->setFlags(autopilots->flags() ^ Qt::ItemIsSelectable);
    autopilots->setExpanded(true);

    foreach(AUV::autopilot_map::value_type i, auv->autopilots) {
        DataStreamTreeItem<float> *autopilot = new DataStreamTreeItem<float>(i.second, autopilots);
        new DataStreamTreeItem<float>(i.second->kP, "kP", autopilot);
        new DataStreamTreeItem<float>(i.second->kI, "kI", autopilot);
        new DataStreamTreeItem<float>(i.second->kD, "kD", autopilot);
        new DataStreamTreeItem<float>(i.second->scale, "scale", autopilot);
        new DataStreamTreeItem<float>(i.second->aP, "aP", autopilot);
        new DataStreamTreeItem<float>(i.second->aI, "aI", autopilot);
        new DataStreamTreeItem<float>(i.second->aD, "aD", autopilot);
        new DataStreamTreeItem<float>(i.second->thr, "thr", autopilot);
    }

    // cameras
    QTreeWidgetItem *cameras = new QTreeWidgetItem(ui->dataStreams);
    cameras->setText(0, "Imaging");
    cameras->setFlags(cameras->flags() ^ Qt::ItemIsSelectable);
    cameras->setExpanded(true);

    foreach(AUV::camera_map::value_type i, auv->cameras) {
        DataStreamTreeItem<Image> * camera = new DataStreamTreeItem<Image>(i.second, cameras);
        if(i.first == CameraID::Sonar)
            new DataStreamTreeItem<sonar_params_t>(boost::shared_static_cast<AUV::Sonar>(auv->cameras[CameraID::Sonar])->params, "Params", camera);
    }

    // misc sensors
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

    // other
    QTreeWidgetItem *other = new QTreeWidgetItem(ui->dataStreams);
    other->setText(0, "Other");
    other->setFlags(sensors->flags() ^ Qt::ItemIsSelectable);
    new DataStreamTreeItem<int32_t>(auv->debug_level, other);


}

DataStreamPicker::~DataStreamPicker(){
    delete ui;
}

void DataStreamPicker::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}




DataStreamDisplayArea::DataStreamDisplayArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QMdiArea(parent),
        CauvInterfaceElement(name, auv, node) {
    this->setAcceptDrops(true);
}

void DataStreamDisplayArea::initialise(){
    m_actions->registerCentralView(this, name());
}

void DataStreamDisplayArea::addWindow(QWidget *content){
    QMdiSubWindow * window = addSubWindow(content);
    window->show();
}

void DataStreamDisplayArea::dropEvent(QDropEvent * event){
    DataStreamDropListener::dropEvent(event);
}

void DataStreamDisplayArea::dragEnterEvent(QDragEnterEvent * event){
    DataStreamDropListener::dragEnterEvent(event);
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream){
    addWindow(new GraphWidget(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<int> > stream){
    addWindow(new GraphWidget(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<float> > stream){
    addWindow(new GraphWidget(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream){
    addWindow(new GraphWidget(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream){
    addWindow(new GraphWidget(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<Image> > stream){
    VideoScreen * vs = new VideoScreen(QString::fromStdString(stream->getName()), this);
    addWindow(vs);
    stream->onUpdate.connect(boost::bind(static_cast<void (VideoScreen::*)(Image&)>(&VideoScreen::setImage), vs, _1));
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

