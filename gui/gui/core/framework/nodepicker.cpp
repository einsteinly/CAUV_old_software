/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "nodepicker.h"
#include "ui_nodepicker.h"

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

    NodePathCompleter(QAbstractItemModel * model, QWidget * parent = NULL) :
            QCompleter(model, parent){
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


void NodePathFilter::setText(QString const& string){
    m_text = string;
    Q_EMIT filterChanged();
}

QString NodePathFilter::getText(){
    return m_text;
}

bool NodePathFilter::containsText(boost::shared_ptr<NodeBase> const& node){
    if(QString::fromStdString(node->nodePath()).contains(getText(), Qt::CaseInsensitive))
        return true;
    BOOST_FOREACH(boost::shared_ptr<NodeBase> const& node, node->getChildren()){
        if(containsText(node)) return true;
    }
    return false;
}

bool NodePathFilter::filter(boost::shared_ptr<NodeBase> const& node){
    return getText().isEmpty() || containsText(node);
}


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



NodePicker::NodePicker(boost::shared_ptr<AUV>  const& auv) :
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

    m_root = boost::make_shared<GroupingNodeTreeItem>(auv);
    ui->dataStreams->addTopLevelItem(m_root.get());
    m_root->setExpanded(true);
    foreach(boost::shared_ptr<NodeBase> child, auv->getChildren()){
        m_root->addNode(child);
    }

    boost::shared_ptr<NodePathFilter> pathFilter = boost::make_shared<NodePathFilter>();
    ui->dataStreams->registerListFilter(pathFilter);

    ui->filter->connect(ui->filter, SIGNAL(textChanged(QString)), pathFilter.get(), SLOT(setText(QString)));
    QCompleter * completer = new NodePathCompleter(ui->dataStreams->model());
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->filter->setCompleter(completer);

    // if the tree changes whilst we're being filtered we need to re-apply filters
    auv->connect(auv.get(), SIGNAL(treeChanged()), ui->dataStreams, SLOT(applyFilters()));

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



NodeListView::NodeListView(QWidget * parent) : QTreeWidget(parent)
{
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

    if(!dsItem){
        error() << "QTreeWidgetItem was not an instance of NodeTreeItemBase";
        return;
    }

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
                debug(3) << "NodeListView passing change to TreeItem to handle";
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


void NodeListView::applyFilters(){
    debug(2) << "applyFilters()";

    for (int i = 0; i < topLevelItemCount(); i++){
        if(NodeTreeItemBase * root = dynamic_cast<NodeTreeItemBase*>(topLevelItem(i))){
            applyFilters(root);
        }
    }
}

void NodeListView::applyFilters(NodeTreeItemBase * item){
    debug(2) << "applyFilters(NodeTreeItemBase *)";

    debug(2) << "filtering" << item->getNode()->nodePath();

    // apply to current node
    if (!applyFilters(item->getNode()))
        item->setHidden(true);
    else item->setHidden(false);

    debug(2) << "applied to current node, recursing on children";

    // recurse on children
    for (int i = 0; i < item->childCount(); i++){
        if(NodeTreeItemBase* nodeItem = dynamic_cast<NodeTreeItemBase*>(item->child(i))){
            applyFilters(nodeItem);
        }
    }
}

bool NodeListView::applyFilters(boost::shared_ptr<NodeBase> const& node){
    debug(2) << "applyFilters(boost::shared_ptr<NodeBase> node)";
    BOOST_FOREACH(boost::shared_ptr<NodeListFilterInterface> const& filter, m_filters){
        // filtering is exclusive, so if any filter says no then the
        // node won't appear in the list
        if (!filter->filter(node)) return false;
    }
    return true;
}

void NodeListView::registerListFilter(boost::shared_ptr<NodeListFilterInterface>  const& filter){
    m_filters.push_back(filter);
    // all filters should have a filterChanged signal, but it's not actually enforced
    // by the interface as Qt doesn't handle multiple inhertiance for QObject
    if(QObject * object = dynamic_cast<QObject *>(filter.get())){
        object->connect(object, SIGNAL(filterChanged()), this, SLOT(applyFilters()));
    }
}
