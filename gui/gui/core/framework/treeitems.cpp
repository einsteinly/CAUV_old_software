#include "treeitems.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cauv;
using namespace cauv::gui;

NodeTreeItemBase::NodeTreeItemBase(boost::shared_ptr<NodeBase> node, QTreeWidgetItem * parent):
        QTreeWidgetItem(parent), m_node(node){
    this->setText(0, QString::fromStdString(node->nodeName()));
    if(node->isMutable()) {
        setTextColor(1, QColor::fromRgb(52, 138, 52));
    }

    node->connect(node.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)),
                  this, SLOT(addNode(boost::shared_ptr<NodeBase>)));
}

void NodeTreeItemBase::updateValue(const QString value) {
    // no update if editing
    if(!(this->flags() & Qt::ItemIsEditable))
        this->setText(1, value);
}

bool NodeTreeItemBase::updateNode(QVariant&) {
    return false;
}
/*

bool NodeTreeItemBase::filter(QString value){

    bool childMatched = false;

    for (int i = 0; i < childCount(); i++){
        QTreeWidgetItem *c = child(i);
        if(NodeTreeItemBase* nodeItem = dynamic_cast<NodeTreeItemBase*>(c)){
            if(nodeItem->filter(value)) {
                childMatched = true;
            }
        }
    }

    if(childMatched || value.isEmpty() ||
       QString::fromStdString(m_node->nodePath()).contains(value, Qt::CaseInsensitive)) {
        info() << m_node->nodePath() << "matched";
        setExpanded(true);
        setHidden(false);
        return true;
    } else {
        setExpanded(false);
        setHidden(true);
        return false;
    }
}
*/
boost::shared_ptr<NodeBase> NodeTreeItemBase::getNode(){
    return m_node;
}

NodeTreeItemBase * NodeTreeItemBase::addNode(boost::shared_ptr<NodeBase> node) {

    debug() << "Adding NodeTreeItem for " << node->nodeName();

    NodeTreeItemBase * item;

    switch (node->type){
    case GuiNodeType::GroupingNode:
        item = new GroupingNodeTreeItem(boost::shared_static_cast<GroupingNode>(node), this);
        break;

    case GuiNodeType::NumericNode:
        item = new NumericNodeTreeItem(boost::shared_static_cast<NumericNode>(node), this);
        break;

    case GuiNodeType::ImageNode:
        item = new ImageNodeTreeItem(boost::shared_static_cast<ImageNode>(node), this);
        break;

    case GuiNodeType::StringNode:
        item = new StringNodeTreeItem(boost::shared_static_cast<StringNode>(node), this);
        break;

    case GuiNodeType::FloatYPRNode:
        item = new FloatYPRNodeTreeItem(boost::shared_static_cast<FloatYPRNode>(node), this);
        break;

    default:
        debug() << "Unsupported node:" << node->nodeName();
        NodeTreeItemBase * item = new NodeTreeItemBase(node, this);
        item->setText(0, item->text(0).append(" [unsupported]"));
        return item;
    }

    // add items for all the nodes children
    foreach(boost::shared_ptr<NodeBase> child, node->getChildren()){
        item->addNode(child);
    }

    return item;
}
