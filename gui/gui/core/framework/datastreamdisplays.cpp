#include "datastreamdisplays.h"
#include "ui_datastreamdisplays.h"

#include "treeitems.h"

#include "../model/model.h"
#include "../widgets/videoscreen.h"
#include "../widgets/graph.h"

#include <common/cauv_utils.h>

#include <QMdiSubWindow>
#include <QModelIndexList>
#include <QKeyEvent>
#include <QCompleter>
#include <QLabel>


using namespace cauv;
using namespace cauv::gui;

class EscapeFilter : public QObject {
public:
    bool eventFilter(QObject *object, QEvent *event){
        if (dynamic_cast<QLineEdit*>(object) && event->type() == QEvent::KeyPress) {
            QLineEdit * edit = (QLineEdit*)object;
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape) {
                edit->clear();
                return true;
            } else
                return false;
        }
        return false;
    }
};


class NodePathCompleter : public QCompleter {

public:

    NodePathCompleter(QAbstractItemModel * model, QWidget * parent = NULL) : QCompleter(model, parent){
    }

    QStringList splitPath(const QString &path) const {
        return path.split("/");
    }

    QString pathFromIndex(const QModelIndex &index) const {
        QTreeWidgetItem *item = static_cast<QTreeWidgetItem*>(index.internalPointer());
        NodeTreeItemBase * dsItem = dynamic_cast<NodeTreeItemBase*>(item);
        if(dsItem)
            return QString::fromStdString(dsItem->getNode()->nodePath());
        else return "";
    }
};

std::vector<boost::shared_ptr<NodeBase> > NodeListView::getDroppedNodes() {
    std::vector<boost::shared_ptr<NodeBase> > streams;

    QModelIndexList items = this->selectedIndexes();
    QModelIndexList::iterator i;
    for (i = items.begin(); i != items.end(); ++i){
        QModelIndex index = (*i);
        if(index.column() == 0) {
            QTreeWidgetItem *item = static_cast<QTreeWidgetItem*>(index.internalPointer());
            NodeTreeItemBase * dsItem = dynamic_cast<NodeTreeItemBase*>(item);
            if(dsItem)
                streams.push_back(dsItem->getNode());
        }
    }
    return streams;
}



NodePicker::NodePicker(boost::shared_ptr<AUV>auv) :
        ui(new Ui::DataStreamPicker())
{
    ui->setupUi(this);

    ui->dataStreams->setRootIsDecorated( true );
    ui->dataStreams->setDragEnabled(true);
    ui->dataStreams->setDropIndicatorShown(true);
    ui->dataStreams->setAcceptDrops(false);
    ui->filter->installEventFilter(new EscapeFilter());


    // setup model data

    debug() << "Initialising data stream list";

    GroupingNodeTreeItem *auvItem = new GroupingNodeTreeItem(auv, NULL);
    ui->dataStreams->addTopLevelItem(auvItem);
    auvItem->setExpanded(true);
    foreach(boost::shared_ptr<NodeBase> child, auv->getChildren()){
        auvItem->addNode(child);
    }

    // dynamic adding of new nodes
    ui->filter->connect(ui->filter, SIGNAL(textChanged(QString)), auvItem, SLOT(filter(QString)));
    QCompleter * completer = new NodePathCompleter(ui->dataStreams->model());
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->filter->setCompleter(completer);

    ui->dataStreams->connect(ui->dataStreams, SIGNAL(onKeyPressed(QKeyEvent*)), this, SLOT(redirectKeyboardFocus(QKeyEvent*)));
}

void NodePicker::redirectKeyboardFocus(QKeyEvent* event){
    if(event->key() == Qt::Key_Escape) {
        ui->filter->setText("");
        return;
    }
    if(!event->text().isEmpty() && event->text().at(0).isLetterOrNumber()) {
        ui->filter->setText("");
        ui->filter->setFocus();
        ui->filter->setText(event->text().simplified());
    }
}

NodePicker::~NodePicker(){
    delete ui;
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



NodeVisualisationArea::NodeVisualisationArea(QWidget * parent) :
        QMdiArea(parent) {
    this->setAcceptDrops(true);
}

void NodeVisualisationArea::addWindow(boost::shared_ptr<QWidget> content){
    AutoDeletingMdiSubWindow * window = new AutoDeletingMdiSubWindow(content);
    this->addSubWindow(window);
    window->show();
}

void NodeVisualisationArea::registerDropHandler(boost::shared_ptr<DropHandler> handler){
    m_handlers.push_back(handler);
}

void NodeVisualisationArea::dropEvent(QDropEvent * event){
    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source) {
        event->acceptProposedAction();
        onDrop(source);
    }
}

void NodeVisualisationArea::dragEnterEvent(QDragEnterEvent * event){
    NodeDragSource * source = dynamic_cast<NodeDragSource*> (event->source());
    if(source) {
        event->acceptProposedAction();
    }
}

void NodeVisualisationArea::onNodeDropped(boost::shared_ptr<NumericNode> node){
    applyHandlers(node);
    //addWindow(boost::make_shared<GraphWidget>(node));
}

void NodeVisualisationArea::onNodeDropped(boost::shared_ptr<ImageNode> node){
    boost::shared_ptr<VideoScreen> video = boost::make_shared<VideoScreen>(QString::fromStdString(node->nodeName()));
    video->connect(node.get(), SIGNAL(onUpdate(image_variant_t)), video.get(), SLOT(setImage(image_variant_t)));
    addWindow(video);
}

void NodeVisualisationArea::onNodeDropped(boost::shared_ptr<FloatYPRNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>(node->findOrCreate<TypedNumericNode<float> >("yaw"));
    widget->addNode(node->findOrCreate<TypedNumericNode<float> >("pitch"));
    widget->addNode(node->findOrCreate<TypedNumericNode<float> >("roll"));
    addWindow(widget);
}

void NodeVisualisationArea::onNodeDropped(boost::shared_ptr<FloatXYZNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>(node->findOrCreate<TypedNumericNode<float> >("x"));
    widget->addNode(node->findOrCreate<TypedNumericNode<float> >("y"));
    widget->addNode(node->findOrCreate<TypedNumericNode<float> >("z"));
    addWindow(widget);
}

void NodeVisualisationArea::onNodeDropped(boost::shared_ptr<GroupingNode> node){
    boost::shared_ptr<GraphWidget> widget = boost::make_shared<GraphWidget>();
    std::vector<boost::shared_ptr<NumericNode> > children = node->getChildrenOfType<NumericNode>();
    if(children.empty()) return;
    foreach(boost::shared_ptr<NumericNode> child, children)
        widget->addNode(child);
    addWindow(widget);
}



NodeListView::NodeListView(QWidget * parent) : QTreeWidget(parent){
    // Because QTreeWidgetItem can't let only one cell in a row be editable we have to hack it
    // around a bit. We set the item to be editable when it's double clicked in the editable
    // cell. This is then removed once the focus is lost. It works but its not a very elegant solution
    // TODO: Better would be to use a full QAbstractItemModel implementation but this is quite a
    // big job as QAbstractItemModels are complicated.
    connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*,int)), this, SLOT(editStarted(QTreeWidgetItem*,int)));
    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(itemEdited(QTreeWidgetItem*,int)));
}

void NodeListView::keyPressEvent(QKeyEvent *event){
    Q_EMIT onKeyPressed(event);
}

void NodeListView::editStarted(QTreeWidgetItem* item, int column){
    if(column == 1){
        cauv::gui::NodeTreeItemBase * dsItem = dynamic_cast<cauv::gui::NodeTreeItemBase*>(item);
        if(dsItem && dsItem->getNode()->isMutable()){
            item->setFlags(item->flags() | Qt::ItemIsEditable);
        }
    }
}

void NodeListView::itemEdited(QTreeWidgetItem* item, int column){
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

