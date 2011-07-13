#include "datastreamdisplays.h"
#include "datastreamdisplay/ui_datastreamdisplays.h"

#include "treeitems.h"

#include <gui/core/model/model.h>
#include <gui/core/widgets/videoscreen.h>
#include "graphs.h"

#include <QMdiSubWindow>
#include <QModelIndexList>


using namespace cauv;
using namespace cauv::gui;

boost::shared_ptr<std::vector<boost::shared_ptr<NodeBase> > > DataStreamList::getDroppedNodes() {
    boost::shared_ptr<std::vector<boost::shared_ptr<NodeBase> > > streams = boost::make_shared<std::vector<boost::shared_ptr<NodeBase> > >();

    QModelIndexList items = this->selectedIndexes();
    QModelIndexList::iterator i;
    for (i = items.begin(); i != items.end(); ++i){
        QModelIndex index = (*i);
        if(index.column() == 0) {
            QTreeWidgetItem *item = static_cast<QTreeWidgetItem*>(index.internalPointer());
            NodeTreeItemBase * dsItem = dynamic_cast<NodeTreeItemBase*>(item);
            if(dsItem)
                streams->push_back(dsItem->getNode());
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

    debug() << "Initialising data stream list";

    GroupingNodeTreeItem *auvItem = new GroupingNodeTreeItem(auv, NULL);
    ui->dataStreams->addTopLevelItem(auvItem);
    auvItem->setExpanded(true);
    foreach(boost::shared_ptr<NodeBase> child, auv->getChildren()){
        auvItem->addNode(child);
    }
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
    NodeDropListener::dropEvent(event);
}

void DataStreamDisplayArea::dragEnterEvent(QDragEnterEvent * event){
    NodeDropListener::dragEnterEvent(event);
}


void DataStreamDisplayArea::onNodeDropped(boost::shared_ptr<NumericNode> node){
    info() << "drop" << node->nodeName();
    addWindow(boost::make_shared<GraphWidget>(node));
}

void DataStreamDisplayArea::onNodeDropped(boost::shared_ptr<ImageNode> node){
    boost::shared_ptr<VideoScreen> video = boost::make_shared<VideoScreen>(QString::fromStdString(node->nodeName()));
    video->connect(node.get(), SIGNAL(onUpdate(image_variant_t)), video.get(), SLOT(setImage(image_variant_t)));
    info() << "drop" << node->nodeName();

}

void DataStreamDisplayArea::onNodeDropped(boost::shared_ptr<FloatYPRNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>(node->findOrCreate<NumericNode>("yaw"));
    widget->addNode(node->findOrCreate<NumericNode>("pitch"));
    widget->addNode(node->findOrCreate<NumericNode>("roll"));
    addWindow(widget);
    info() << "drop" << node->nodeName();
}

void DataStreamDisplayArea::onNodeDropped(boost::shared_ptr<FloatXYZNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>(node->findOrCreate<NumericNode>("x"));
    widget->addNode(node->findOrCreate<NumericNode>("y"));
    widget->addNode(node->findOrCreate<NumericNode>("z"));
    addWindow(widget);
    info() << "drop" << node->nodeName();
}

void DataStreamDisplayArea::onNodeDropped(boost::shared_ptr<GroupingNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>();
    foreach(boost::shared_ptr<NumericNode> child, node->getChildrenOfType<NumericNode>())
        widget->addNode(child);
    info() << "drop" << node->nodeName();
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
        cauv::gui::NodeTreeItemBase * dsItem = dynamic_cast<cauv::gui::NodeTreeItemBase*>(item);
        if(dsItem && dsItem->getNode()->isMutable()){
            item->setFlags(item->flags() | Qt::ItemIsEditable);
        }
    }
}

void DataStreamList::itemEdited(QTreeWidgetItem* item, int column){
    cauv::gui::NodeTreeItemBase * dsItem = dynamic_cast<cauv::gui::NodeTreeItemBase*>(item);

    // 1 is the editable cell that stores the value
    if(column == 1 && dsItem && dsItem->getNode()->isMutable()){
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
                if(dsItem->updateNode(v)){
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
