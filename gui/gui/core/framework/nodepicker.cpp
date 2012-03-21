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

#include <QKeyEvent>
#include <QCompleter>
#include <QPushButton>
#include <QLayout>

#include <debug/cauv_debug.h>

#include "model/nodes/numericnode.h"
#include "delegates.h"
#include "model/model.h"

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
        Node *node = static_cast<Node*>(index.internalPointer());
        return QString::fromStdString(node->nodePath()).remove(QRegExp("^/"));
    }
};


NodePathFilter::NodePathFilter(QObject * parent) : QObject(parent), m_text(""){
}

void NodePathFilter::setText(QString const& string){
    m_text = string;
    Q_EMIT filterChanged();
}

QString NodePathFilter::getText(){
    return m_text;
}

bool NodePathFilter::containsText(boost::shared_ptr<Node> const& node){
    debug(7) << "NODE = " << node;
    debug(7) << "PATH = " << node->nodePath();
    debug(7) << "TEXT = " << getText().toStdString();

    if(QString::fromStdString(node->nodePath()).contains(getText(), Qt::CaseInsensitive))
        return true;

    // might match somewhere in a child nodes path
    foreach(boost::shared_ptr<Node> const& child, node->getChildren()){
        if(containsText(child)) return true;
    }

    return false;
}

bool NodePathFilter::filter(boost::shared_ptr<Node> const& node){
    return getText().isEmpty() || containsText(node);
}


NodeExclusionFilter::NodeExclusionFilter(QObject *parent) : QObject(parent){
}

bool NodeExclusionFilter::filter(boost::shared_ptr<Node> const& node){
    foreach(boost::shared_ptr<Node> n, m_nodes){
        if(n.get() == node.get()) return false;
    }
    return true;
}

void NodeExclusionFilter::addNode(boost::shared_ptr<Node> node){
    m_nodes.push_back(node);
}


NodePicker::NodePicker(boost::shared_ptr<NodeItemModel> const& root) :
     m_root(root), ui(new Ui::NodePicker())
{
    ui->setupUi(this);

    ui->view->setRootIsDecorated( true );
    ui->view->setDragEnabled(true);
    ui->view->setDropIndicatorShown(true);
    ui->view->setAcceptDrops(false);
    ui->filter->installEventFilter(new EscapeFilter());
    ui->view->setModel(root.get());
    ui->view->addNumericDelegateToColumn(1);

    QHBoxLayout * layout = new QHBoxLayout(ui->filter);
    QPushButton * button = new QPushButton("X");
    button->setCursor(Qt::PointingHandCursor);
    button->connect(button, SIGNAL(clicked()), ui->filter, SLOT(clear()));
    layout->addWidget(button, 0, Qt::AlignLeft);
    layout->setMargin(0);
    layout->setContentsMargins(0, 0, 5, 0);

    // The list supports node filtering as there's potentially a lot of nodes
    boost::shared_ptr<NodePathFilter> pathFilter = boost::make_shared<NodePathFilter>();
    ui->view->registerListFilter(pathFilter);
    ui->filter->connect(ui->filter, SIGNAL(textChanged(QString)), pathFilter.get(), SLOT(setText(QString)));

    // and auto completion of the path filter
    QCompleter * completer = new NodePathCompleter(ui->view->model());
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->filter->setCompleter(completer);

    // if the tree changes whilst we're being filtered we need to re-apply filters
    QModelIndex index = root->index(0,0,QModelIndex());
    Node * rootNode = static_cast<Node*>(index.internalPointer());
    root->connect(rootNode, SIGNAL(structureChanged()), ui->view, SLOT(applyFilters()));

    // if the list is in focus (but keystrokes are not swalled by an edit box) then
    // redirect focus so the filter gets the key events
    ui->view->connect(ui->view, SIGNAL(onKeyPressed(QKeyEvent*)), this, SLOT(redirectKeyboardFocus(QKeyEvent*)));

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

void NodePicker::registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter){
   ui->view->registerListFilter(filter);
}

NodePicker::~NodePicker(){
    delete ui;
}



NodeTreeView::NodeTreeView(QWidget *) {
    header()->hide();
    setColumnWidth(0, 200);
    setIndentation(15);
    setRootIsDecorated(false);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setAllColumnsShowFocus(true);
    setAnimated(true);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

    //this->setMouseTracking(true);
    //connect(this, SIGNAL(entered(QModelIndex)), this, SLOT(forceEdit(QModelIndex)));
    connect(this, SIGNAL(clicked(QModelIndex)), this, SLOT(forceEdit(QModelIndex)));
    connect(this, SIGNAL(clicked(QModelIndex)), this, SLOT(toggleExpanded(QModelIndex)));
}


void NodeTreeView::addNumericDelegateToColumn(int col){
    debug() << "NodeTreeView::addNumericDelegateToColumn()";
    NodeDelegateMapper *delegate = new NodeDelegateMapper(this);
    setItemDelegateForColumn(col, delegate);
    delegate->registerDelegate(nodeType<NumericNodeBase>(), boost::make_shared<NumericDelegate>(this));
}

void NodeTreeView::forceEdit(QModelIndex const& index){
    if(index.column() == 1) {
        if(index.flags() & Qt::ItemIsEditable)
            edit(index);
    }
}

void NodeTreeView::toggleExpanded(QModelIndex const& index){
    setExpanded(index, !isExpanded(index));
}

void NodeTreeView::keyPressEvent(QKeyEvent *event){
    Q_EMIT onKeyPressed(event);
    QTreeView::keyPressEvent(event);
}


void NodeTreeView::applyFilters(){
    debug(8) << "applyFilters()";
    applyFilters(model()->index(0,0,QModelIndex()));
}

void NodeTreeView::applyFilters(QModelIndex const& index){
    debug(8) << "applyFilters(QModelIndex const&)";

    boost::shared_ptr<Node> node = static_cast<Node*>(index.internalPointer())->shared_from_this();

    debug(8) << "filtering" << node->nodePath();

    // apply to current node
    if (!applyFilters(node))
        this->setRowHidden(index.row(), index.parent(), true);
    else this->setRowHidden(index.row(), index.parent(), false);

    // recurse on children
    for (int i = 0; i < model()->rowCount(index); i++) {
        applyFilters(index.child(i, 0));
    }
}

bool NodeTreeView::applyFilters(boost::shared_ptr<Node> const& node){
    debug(8) << "applyFilters(boost::shared_ptr<Node> node)";
    foreach(boost::shared_ptr<NodeFilterInterface> const& filter, m_filters){
        // filtering is exclusive, so if any filter says no then the
        // node won't appear in the list
        if (!filter->filter(node)) return false;
    }
    return true;
}

void NodeTreeView::registerListFilter(boost::shared_ptr<NodeFilterInterface>  const& filter){
    m_filters.push_back(filter);
    // all filters should have a filterChanged signal, but it's not actually enforced
    // by the interface as Qt doesn't handle multiple inhertiance for QObject
    if(QObject * object = dynamic_cast<QObject *>(filter.get())){
        object->connect(object, SIGNAL(filterChanged()), this, SLOT(applyFilters()));
    }
}
