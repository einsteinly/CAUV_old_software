
#include <boost/signals2.hpp>

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

    qRegisterMetaType<Image>("Image");

    connect(this, SIGNAL(iconUpdated(int, Image)), this, SLOT(updateIcon(int, Image)));
    connect(this, SIGNAL(valueUpdated(QString)), this, SLOT(updateValue(QString)));
}

void DataStreamTreeItemBase::updateIcon(int cell, QImage &image){
    this->setIcon(cell, QIcon(QPixmap::fromImage(image)));
}

void DataStreamTreeItemBase::updateIcon(int cell, const Image &image){
    try {
        cv::Mat mat_rgb;
        cv::cvtColor(image.cvMat(), mat_rgb, CV_BGR2RGB);

        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols,
                               mat_rgb.rows, QImage::Format_RGB888);

        this->updateIcon(cell, qImage);

    } catch (...){
        error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__;
    }
}

void DataStreamTreeItemBase::updateValue(const QString value) {
    this->setText(1, value);
}

boost::shared_ptr<DataStreamBase> DataStreamTreeItemBase::getDataStreamBase(){
    return m_stream;
}




template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value){
    std::stringstream stream;
    stream << (int)value;
    Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
}

template<> void DataStreamTreeItem<Image>::onChange(const Image value){
    // Add a fancy icon in the data stream list with the current view from the camera.
    Q_EMIT this->iconUpdated(1, value);
}

template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant &value){
    return boost::lexical_cast<int>(value.toString().toStdString());
}

template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& ){
    // TODO: implement, should recognise something like (1.0, 2.0, 3.0)
    return floatYPR();
}

template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& ){
    // this shouldn't ever be used
    throw new boost::bad_lexical_cast;
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

    //
    // motors
    //
    QTreeWidgetItem *motors = new QTreeWidgetItem(ui->dataStreams);
    motors->setText(0, "Motors");
    motors->setFlags(motors->flags() ^ Qt::ItemIsSelectable);
    motors->setExpanded(true);

    foreach(AUV::motor_map::value_type i, auv->motors) {
        new DataStreamTreeItem<int8_t>(i.second, motors);
    }

    //
    // autopilots
    //
    QTreeWidgetItem *autopilots = new QTreeWidgetItem(ui->dataStreams);
    autopilots->setText(0, "Autopilots");
    autopilots->setFlags(autopilots->flags() ^ Qt::ItemIsSelectable);
    autopilots->setExpanded(true);

    foreach(AUV::autopilot_map::value_type i, auv->autopilots) {
        DataStreamTreeItem<float> *autopilot = new DataStreamTreeItem<float>(i.second, autopilots);
        (new DataStreamTreeItem<float>(i.second->kP, autopilot))->setText(0, "kP");
        (new DataStreamTreeItem<float>(i.second->kI, autopilot))->setText(0, "kI");
        (new DataStreamTreeItem<float>(i.second->kD, autopilot))->setText(0, "kD");
        (new DataStreamTreeItem<float>(i.second->aP, autopilot))->setText(0, "aP");
        (new DataStreamTreeItem<float>(i.second->aI, autopilot))->setText(0, "aI");
        (new DataStreamTreeItem<float>(i.second->aD, autopilot))->setText(0, "aD");
        (new DataStreamTreeItem<float>(i.second->thr, autopilot))->setText(0, "thr");
        (new DataStreamTreeItem<float>(i.second->scale, autopilot))->setText(0, "scale");
        (new DataStreamTreeItem<float>(i.second->actual, autopilot))->setText(0, "actual");
    }

    //
    // imaging devices
    //
    QTreeWidgetItem *cameras = new QTreeWidgetItem(ui->dataStreams);
    cameras->setText(0, "Imaging");
    cameras->setFlags(cameras->flags() ^ Qt::ItemIsSelectable);
    cameras->setExpanded(true);

    foreach(AUV::camera_map::value_type i, auv->cameras) {
        DataStreamTreeItem<Image> * camera = new DataStreamTreeItem<Image>(i.second, cameras);

        // special case for sonars as they have params
        if(dynamic_cast<AUV::Sonar*>(i.second.get())) {
            boost::shared_ptr<AUV::Sonar> sonar = boost::shared_static_cast<AUV::Sonar>(auv->cameras[CameraID::Sonar]);

            new DataStreamTreeItem<int>(sonar->direction, camera);
            new DataStreamTreeItem<int>(sonar->angularRes, camera);
            new DataStreamTreeItem<int>(sonar->radialRes, camera);
            new DataStreamTreeItem<int>(sonar->gain, camera);
            new DataStreamTreeItem<int>(sonar->range, camera);
            new DataStreamTreeItem<int>(sonar->width, camera);
        }
    }

    //
    // sensors
    //
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

    //
    // other
    //
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




class AutoDeletingMdiSubWindow : public QMdiSubWindow {
public:
    AutoDeletingMdiSubWindow(boost::shared_ptr<QWidget> widget) : m_widget(widget) {
        this->setWidget(widget.get());
        this->setAttribute(Qt::WA_DeleteOnClose);
    }

    void closeEvent(QCloseEvent *){
        // before the window closes we need to kill the widget
        // this will in turn disconnect any signals that may still try
        // to use the window once it's been closed, causing a segfault
        m_widget.reset();
    }

    boost::shared_ptr<QWidget> m_widget;
};



DataStreamDisplayArea::DataStreamDisplayArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QMdiArea(parent),
        CauvInterfaceElement(name, auv, node) {
    this->setAcceptDrops(true);
}

void DataStreamDisplayArea::initialise(){
    m_actions->registerCentralView(this, CauvInterfaceElement::name());
}

void DataStreamDisplayArea::addWindow(boost::shared_ptr<QWidget> content){
    AutoDeletingMdiSubWindow * window = new AutoDeletingMdiSubWindow(content);
    this->addSubWindow(window);
    window->show();
}

void DataStreamDisplayArea::dropEvent(QDropEvent * event){
    DataStreamDropListener::dropEvent(event);
}

void DataStreamDisplayArea::dragEnterEvent(QDragEnterEvent * event){
    DataStreamDropListener::dragEnterEvent(event);
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream){
    addWindow(boost::make_shared<GraphWidget>(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<int> > stream){
    addWindow(boost::make_shared<GraphWidget>(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<float> > stream){
    addWindow(boost::make_shared<GraphWidget>(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream){
    addWindow(boost::make_shared<GraphWidget>(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream){
    addWindow(boost::make_shared<GraphWidget>(stream));
}

void DataStreamDisplayArea::onStreamDropped(boost::shared_ptr<DataStream<Image> > stream){
    boost::shared_ptr<VideoScreen> vs = boost::make_shared<VideoScreen>(QString::fromStdString(stream->getName()));
    addWindow(vs);
    stream->onUpdate.connect(DataStream<Image>::signal_type::slot_type(&VideoScreen::setImage, vs.get(), _1).track(vs));
}




DataStreamList::DataStreamList(QWidget * parent) : QTreeWidget(parent){
    // Because QTreeWidgetItem can't let only one cell in a row be editable we have to hack it
    // around a bit. We set the item to be editable when it's double clicked in the editable
    // cell. This is then removed once the focus is lost. It works but its not a very elegant solution
    // TODO: Better would be to use a full QAbstractItemModel implementation but this is quite a
    // big job as QAbstractItemModels are complicated.
    connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), this, SLOT(editStarted(QTreeWidgetItem*,int)));
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
                // disable editing again
                // do it before the item is actually updated, otherwise this method will get
                // called again when stream->set(...) is called.
                item->setFlags(item->flags() & (~Qt::ItemIsEditable));

                // do the update and check the result
                if(dsItem->updateStream(v)){
                    // reset the background if all is ok
                    item->setBackground(1, QBrush());
                } else  {
                    // set different colours to show somethings not right
                    QBrush b(Qt::DiagCrossPattern);
                    b.setColor(QColor::fromRgb(224, 128, 128));
                    item->setBackground(1, b);
                }
            }
        }
    }
}
