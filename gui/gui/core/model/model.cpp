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

#include "model.h"

#include <QUrl>

#include "node.h"

using namespace cauv;
using namespace cauv::gui;


NodeUpdateModelNotfication::NodeUpdateModelNotfication(NodeItemModel * model) : m_model(model) {
    debug(8) << "NodeUpdateModelNotfication()";
}

void NodeUpdateModelNotfication::update(){
    if (Node * n = dynamic_cast<Node*>(this->sender())) {
        boost::shared_ptr<Node> node = n->shared_from_this();
        QModelIndex index = m_model->indexFromNode(node);
        Q_EMIT m_model->dataChanged(index, index);
    }
}


NodeItemModel::NodeItemModel(boost::shared_ptr<Node> root, QObject * parent) :
    QAbstractItemModel(parent), m_root(root), m_updater(this){
    connect(root.get(), SIGNAL(structureChanged()), this, SIGNAL(layoutChanged()));
    connectUpdater(root);
}

QModelIndex NodeItemModel::indexFromNode(boost::shared_ptr<Node> node) const{
    return createIndex(node->row(), 1, node.get());
}

boost::shared_ptr<Node> NodeItemModel::nodeFromIndex(QModelIndex const& index) const{
    if(index.internalPointer()) {
        Node* node = static_cast<Node *>(index.internalPointer());
        return node->shared_from_this();
    } else throw std::runtime_error("Invalid node pointer in model!");
}

Qt::ItemFlags NodeItemModel::flags(const QModelIndex &index) const{
    boost::shared_ptr<Node> node = nodeFromIndex(index);
    if(index.column()!=0 && node->isMutable()) // column 0 is the node name which can't be chaged
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
    else return QAbstractItemModel::flags(index) | Qt::ItemIsDragEnabled;
}

QVariant NodeItemModel::data(const QModelIndex & index, int role) const {
    if(!index.isValid()) return QVariant();
    boost::shared_ptr<Node> node = nodeFromIndex(index);
    
    // !!! no index validation? /jc
    switch (role){
    case Qt::EditRole:
    case Qt::DisplayRole:
        if (index.column() == 0)
            return QVariant(QString::fromStdString(node->nodeName()));
        else return node->get();
        break;
    case Qt::ToolTipRole:
        return QVariant(QString::fromStdString(node->nodePath()));
        break;
    default: break;
    }
    return QVariant();
}

QVariant NodeItemModel::headerData(int section, Qt::Orientation orientation, int role) const {
    Q_UNUSED(orientation);
    Q_UNUSED(section);
    Q_UNUSED(role);
    return QVariant();
}

bool NodeItemModel::setData(const QModelIndex & index, const QVariant & value, int role) {
    if(!index.isValid()) return false;
    boost::shared_ptr<Node> node = nodeFromIndex(index);

    switch (role){
    case Qt::EditRole:
        if (node->set(value)) {
            Q_EMIT dataChanged(index, index);
            return true;
        }
    default: return false;
    }
}

QMimeData * NodeItemModel::mimeData(const QModelIndexList &indexes) const {

    QMimeData *mimeData = new QMimeData;
    QList<QUrl> urls;

    foreach (QModelIndex const & index, indexes){
        if(index.column() == 0) {
            // get path from the node
            boost::shared_ptr<Node> node = nodeFromIndex(index);
            QUrl url = QUrl();
            url.setScheme("varstream");
            boost::shared_ptr<Vehicle> vehicleNode = node->getClosestParentOfType<Vehicle>();
            url.setHost(QString::fromStdString(vehicleNode->nodeName()));
            url.setPath(QString::fromStdString(node->nodePath()).remove(url.host().prepend("/")));
            urls.append(url);
            debug(5) << url.toString().toStdString() << "added to drag";
        }
    }

    mimeData->setUrls(urls);
    return mimeData;
}

int NodeItemModel::rowCount ( const QModelIndex & parent ) const {
    boost::shared_ptr<Node> parentItem;
    if (!parent.isValid())
        parentItem = m_root;
    else parentItem = nodeFromIndex(parent);

    return parentItem->getChildren().size();
}

int NodeItemModel::columnCount(const QModelIndex &/*parent*/) const {
    // !!! todo: variable column sizes?
    return 2;
}

QModelIndex NodeItemModel::parent(const QModelIndex &child) const {
    if (!child.isValid())
        return QModelIndex();

    boost::shared_ptr<Node> childNode = nodeFromIndex(child);
    try {
        boost::shared_ptr<Node> parent = childNode->getParent();
        return indexFromNode(parent);
    } catch (std::out_of_range ex){
        // parent is missing
        return QModelIndex();
    }
}

QModelIndex NodeItemModel::index(int row, int column, const QModelIndex &parent) const {
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    boost::shared_ptr<Node> parentNode;
    if(!parent.isValid())
        parentNode = m_root;
    else parentNode = nodeFromIndex(parent);

    try {
        boost::shared_ptr<Node> childNode = parentNode->getChildren().at(row);
        return createIndex(row, column, childNode.get());
    } catch (std::out_of_range){
        return QModelIndex();
    }
}

void NodeItemModel::connectUpdater(boost::shared_ptr<Node> node){
    node->connect(node.get(), SIGNAL(onUpdate()), &m_updater, SLOT(update()));
    foreach (boost::shared_ptr<Node> const & child, node->getChildren()){
        connectUpdater(child);
    }
    connect(node.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(connectUpdater(boost::shared_ptr<Node>)));
}

boost::shared_ptr<Node> NodeItemModel::rootNode(){
    return m_root;
}
