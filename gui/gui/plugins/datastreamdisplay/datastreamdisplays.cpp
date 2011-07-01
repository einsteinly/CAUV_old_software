#include "datastreamdisplays.h"
#include "datastreamdisplay/ui_datastreamdisplays.h"

#include "treeitems.h"

#include <gui/core/widgets/videoscreen.h>
#include "graphs.h"

#include <QMdiSubWindow>
#include <QModelIndexList>

using namespace cauv;


boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > DataStreamList::getDataStreams() {
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



DataStreamPicker::DataStreamPicker() :
        ui(new Ui::DataStreamPicker())
{
    ui->setupUi(this);

    ui->dataStreams->setRootIsDecorated( true );
    ui->dataStreams->setDragEnabled(true);
    ui->dataStreams->setDropIndicatorShown(true);
    ui->dataStreams->setAcceptDrops(false);

    m_docks[this] = Qt::LeftDockWidgetArea;
    m_tabs.append(new DataStreamDisplayArea());

}

void DataStreamPicker::initialise(boost::shared_ptr<AUV>auv, boost::shared_ptr<CauvNode>node) {
    CauvBasicPlugin::initialise(auv, node);

    // set up the categories

    error() << "Initisaliing data strema plugin";

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
        (new DataStreamTreeItem<float>(i.second->maxError, autopilot))->setText(0, "maxError");
        (new DataStreamTreeItem<float>(i.second->scale, autopilot))->setText(0, "scale");
        (new DataStreamTreeItem<float>(i.second->actual, autopilot))->setText(0, "actual");

        QTreeWidgetItem *demands = new QTreeWidgetItem(autopilot);
        demands->setText(0, "demands");
        demands->setFlags(demands->flags() ^ Qt::ItemIsSelectable);
        (new DataStreamTreeItem<float>(i.second->demand->prop, demands))->setText(0, "prop");
        (new DataStreamTreeItem<float>(i.second->demand->hbow, demands))->setText(0, "hbow");
        (new DataStreamTreeItem<float>(i.second->demand->vbow, demands))->setText(0, "vbow");
        (new DataStreamTreeItem<float>(i.second->demand->hstern, demands))->setText(0, "hstern");
        (new DataStreamTreeItem<float>(i.second->demand->vstern, demands))->setText(0, "vstern");

        (new DataStreamTreeItem<float>(i.second->mv, autopilot))->setText(0, "mv");
        (new DataStreamTreeItem<float>(i.second->error, autopilot))->setText(0, "error");
        (new DataStreamTreeItem<float>(i.second->derror, autopilot))->setText(0, "derror");
        (new DataStreamTreeItem<float>(i.second->ierror, autopilot))->setText(0, "ierror");

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
    DataStreamTreeItem<floatYPR> * orientation = new DataStreamTreeItem<floatYPR>(auv->sensors.orientation->combined, sensors);
    new DataStreamTreeItem<float>(auv->sensors.orientation->yaw, orientation);
    new DataStreamTreeItem<float>(auv->sensors.orientation->pitch, orientation);
    new DataStreamTreeItem<float>(auv->sensors.orientation->roll, orientation);


    QTreeWidgetItem *battery = new QTreeWidgetItem(ui->dataStreams);
    battery->setText(0, "Battery");
    battery->setFlags(battery->flags() ^ Qt::ItemIsSelectable);
    battery->setExpanded(true);

    new DataStreamTreeItem<float>(auv->sensors.esitmate_current, battery);
    new DataStreamTreeItem<float>(auv->sensors.estimate_total, battery);
    new DataStreamTreeItem<float>(auv->sensors.fraction_remaining, battery);


    //
    // debug graphs
    //
    m_debug = new QTreeWidgetItem(ui->dataStreams);
    m_debug->setText(0, "Debug");
    m_debug->setFlags(m_debug->flags() ^ Qt::ItemIsSelectable);
    m_auv->debug.new_graph_stream->onUpdate.connect(boost::bind(&DataStreamPicker::onNewGraphableStream, this, _1));

    //
    // other
    //
    QTreeWidgetItem *other = new QTreeWidgetItem(ui->dataStreams);
    other->setText(0, "Other");
    other->setFlags(other->flags() ^ Qt::ItemIsSelectable);
    new DataStreamTreeItem<int32_t>(auv->debug_level, other);

}

void DataStreamPicker::onNewGraphableStream(boost::shared_ptr<DataStream<float> > stream){
    new DataStreamTreeItem<float>(stream, m_debug);
}

DataStreamPicker::~DataStreamPicker(){
    delete ui;
}

const QString DataStreamPicker::name() const{
    return QString("Data Streams");
}

const QList<QString> DataStreamPicker::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("gui"));
    groups.push_back(QString("telemetry"));
    groups.push_back(QString("control"));
    groups.push_back(QString("state"));
    groups.push_back(QString("image"));
    groups.push_back(QString("sonarout"));
    groups.push_back(QString("sonarctl"));
    groups.push_back(QString("pressure"));
    // TODO add all groups...
    return groups;
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



DataStreamDisplayArea::DataStreamDisplayArea(QWidget * parent) :
        QMdiArea(parent) {
    this->setAcceptDrops(true);
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



Q_EXPORT_PLUGIN2(cauv_dsdplugin, DataStreamPicker)
