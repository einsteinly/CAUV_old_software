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

#include "../model/messagegenerators.h"
#include "../model/nodes/numericnode.h"
#include "../model/nodes/groupingnode.h"

using namespace cauv;
using namespace cauv::gui;


RedHerring::RedHerring(std::string name) : Vehicle(name) {
    // don't populate anything in here as there isn't a shared pointer to
    // this object yet. We need to wait until after it's been fully constructed
}

void RedHerring::initialise() {
    // when a child is added to the motors group we want to add a message generator for it
    boost::shared_ptr<GroupingNode> motors = findOrCreate<GroupingNode>("motors");
    connect(motors.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(setupMotor(boost::shared_ptr<Node>)));

    // same for autopilots
    boost::shared_ptr<GroupingNode> autopilots = findOrCreate<GroupingNode>("autopilots");
    connect(autopilots.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(setupAutopilot(boost::shared_ptr<Node>)));

}

void RedHerring::setupMotor(boost::shared_ptr<Node> node){
    try {
        boost::shared_ptr<NumericNode<int> > motor = node->to<NumericNode<int> >();
        motor->setMax(127);
        motor->setMin(-127);
        motor->setMutable(true);
        addGenerator(boost::make_shared<MotorMessageGenerator>(boost::static_pointer_cast<Vehicle>(shared_from_this()), motor));
    } catch (std::runtime_error){
        warning() << node->nodePath() << " should be a NumericNode";
    }
}

void RedHerring::setupAutopilot(boost::shared_ptr<Node> node){

    addGenerator(boost::make_shared<AutopilotMessageGenerator>(boost::static_pointer_cast<Vehicle>(shared_from_this()), node));
    boost::shared_ptr<NumericNode<float> > target = node->findOrCreate<NumericNode<float> >("target");
    target->setMutable(true);
    node->findOrCreate<NumericNode<bool> >("enabled")->setMutable(true);

    // target params
    float min, max; bool wraps; std::string units;

    Controller::e id = boost::get<Controller::e>(node->nodeId());

    // params vary for each autopilot
    switch(id){
    case Controller::Bearing:
        min=0; max=360; wraps=true; units="°";
        break;
    case Controller::Pitch:
        min=-180; max=180; wraps=true; units="°";
        break;
    case Controller::Depth:
        min=-1; max=5; wraps=false; units="m";
        break;
    default: return;
    }

    target->setMin(min);
    target->setMax(max);
    target->setWraps(wraps);
    target->setUnits(units);
    target->setPrecision(3);
}



NodeItemModel::NodeItemModel(boost::shared_ptr<Node> root, QObject * parent) :
    QAbstractItemModel(parent), m_root(root){
    connect(root.get(), SIGNAL(structureChanged()), this, SIGNAL(layoutChanged()));
    createUpdater(root);
}

QModelIndex NodeItemModel::indexFromNode(boost::shared_ptr<Node> node) const{
    return createIndex(node->row(), 1, node.get());
}

boost::shared_ptr<Node> NodeItemModel::nodeFromIndex(QModelIndex const& index) const{
    return static_cast<Node *>(index.internalPointer())->shared_from_this();
}

Qt::ItemFlags NodeItemModel::flags(const QModelIndex &index) const{
    boost::shared_ptr<Node> node = nodeFromIndex(index);
    if(index.column()!=0 && node->isMutable()) // column 0 is the node name which can't be chaged
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
    else return QAbstractItemModel::flags(index) | Qt::ItemIsDragEnabled;
}

QVariant NodeItemModel::data(const QModelIndex & index, int role) const {
    boost::shared_ptr<Node> node = nodeFromIndex(index);

    switch (role){
    case Qt::EditRole:
    case Qt::DisplayRole:
        if (index.column() == 0)
            return QVariant(QString::fromStdString(node->nodeName()));
        else return node->get();
        break;
    case Qt::DecorationRole:
        break;
    case Qt::ToolTipRole:
        return QVariant(QString::fromStdString(node->nodePath()));
        break;
    case Qt::StatusTipRole:
        break;
    case Qt::WhatsThisRole:
        break;
    case Qt::SizeHintRole:
        break;
    case Qt::FontRole:
        break;
    case Qt::TextAlignmentRole:
        break;
    case Qt::BackgroundRole:
        break;
    case Qt::ForegroundRole:
        break;
    case Qt::CheckStateRole:
        break;
    case Qt::AccessibleTextRole:
        break;
    case Qt::AccessibleDescriptionRole:
        break;
    default: break;
    }
    return QVariant();
}

QVariant NodeItemModel::headerData(int section, Qt::Orientation orientation, int role) const {

    Q_UNUSED(orientation);

    switch (role){
    case Qt::ToolTipRole:
    case Qt::EditRole:
    case Qt::DisplayRole:
        if (orientation == Qt::Horizontal && section == 0) return "Name";
        if (orientation == Qt::Horizontal && section == 1) return "Value";
        else return QString("%1").arg(section);
        break;
    case Qt::DecorationRole:
        break;
    case Qt::StatusTipRole:
        break;
    case Qt::WhatsThisRole:
        break;
    case Qt::SizeHintRole:
        break;
    case Qt::FontRole:
        break;
    case Qt::TextAlignmentRole:
        break;
    case Qt::BackgroundRole:
        break;
    case Qt::ForegroundRole:
        break;
    case Qt::CheckStateRole:
        break;
    case Qt::AccessibleTextRole:
        break;
    case Qt::AccessibleDescriptionRole:
        break;
    default: break;
    }
    return QVariant();
}

bool NodeItemModel::setData(const QModelIndex & index, const QVariant & value, int role) {

    boost::shared_ptr<Node> node = nodeFromIndex(index);

    switch (role){
    case Qt::DisplayRole:
        // can't be edited
        break;
    case Qt::EditRole:
        if (node->set(value))
        {
            Q_EMIT dataChanged(index, index);
            return true;
        }
        break;
    default: break;
    }
    return false;
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
    else
        parentItem = nodeFromIndex(parent);

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

void NodeItemModel::createUpdater(boost::shared_ptr<Node> node){
    // !!! todo: fix memory leak
    QModelIndex index = indexFromNode(node);
    ModelIndexUpdateNotfication * updater = new ModelIndexUpdateNotfication(index, index);
    node->connect(node.get(), SIGNAL(onUpdate()), updater, SLOT(update()));
    this->connect(updater, SIGNAL(onUpdate(QModelIndex,QModelIndex)), this, SIGNAL(dataChanged(QModelIndex, QModelIndex)));

    foreach (boost::shared_ptr<Node> const & child, node->getChildren()){
        createUpdater(child);
    }

    connect(node.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)), this, SLOT(createUpdater(boost::shared_ptr<Node>)));
}

