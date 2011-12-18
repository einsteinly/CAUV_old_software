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

#include "node.h"

#include <QMetaType>

using namespace cauv;
using namespace cauv::gui;


int node_types::count = 0;
std::map<std::string, int> node_types::typeMap;


Node::Node(nid_t const& id, node_type t) :
        type(t), m_parent(), m_id(id), m_mutable(false) {

    qRegisterMetaType<boost::shared_ptr<Node> >("boost::shared_ptr<Node>");
}

Node::~Node(){
    debug(2) << "~NodeBase" << nodeName();
}

std::string Node::nodeName() const {
    return boost::apply_visitor(id_to_name(), m_id);
}

nid_t Node::nodeId() const {
    return m_id;
}

std::string Node::nodePath() const {
    std::stringstream stream;

    if(!m_parent.expired()) {
        boost::shared_ptr<Node> parent = m_parent.lock();
        if(parent)
            stream << parent->nodePath() << "/" << nodeName();
    }
    else stream << nodeName();

    return stream.str();
}

void Node::addChild(boost::shared_ptr<Node> const& child){
    if(boost::shared_ptr<Node> parent = child->m_parent.lock())
    {
        std::stringstream str;
        str << "Node already has a parent: " << parent->nodeName();
        throw std::runtime_error(str.str());
    }

    child->m_parent = boost::weak_ptr<Node>(shared_from_this());
    m_children.push_back(child);
    m_id_map[child->nodeId()] = child;
    // also add the pretty name as a key to this node
    // as this is used for reverse lookups (i.e. prop -> MotorID::Prop)
    m_id_map[child->nodeName()] = child;

    // make sure all the objects belong to the thread of the root item
    // children get created in many threads (e.g. the messaging thread)
    // and these threads don't have event loops so signals won't work
    child->moveToThread(this->thread());

    // propagate changes upwards
    child->connect(child.get(), SIGNAL(onBranchChanged()), this, SIGNAL(onBranchChanged()));
    child->connect(child.get(), SIGNAL(structureChanged()), this, SIGNAL(structureChanged()));

    Q_EMIT nodeAdded(child);
    Q_EMIT structureChanged();
}


void Node::removeChild(boost::shared_ptr<Node> const& child){

    //check it's actually a child of this node (throws exception if not)
    find<Node>(child->nodeId());

    // clear parent from child
    child->m_parent.reset();
    m_children.erase(std::find(m_children.begin(), m_children.end(), child));
    m_id_map.erase(child->nodeId());
    m_id_map.erase(child->nodeName());

    // disconnect propagation signals
    child->disconnect(child.get(), SIGNAL(onBranchChanged()), this, SIGNAL(onBranchChanged()));
    child->disconnect(child.get(), SIGNAL(structureChanged()), this, SIGNAL(structureChanged()));

    Q_EMIT nodeRemoved(child);
    Q_EMIT structureChanged();
}

const Node::children_list_t Node::getChildren() const {
    return m_children;
}

bool Node::isMutable() const{
    return m_mutable;
}

void Node::setMutable(bool mut){
    m_mutable = mut;
}

boost::shared_ptr<Node> Node::getRoot() {
    boost::shared_ptr<Node> node = shared_from_this();

    while(node->m_parent.lock())
        node = node->m_parent.lock();

    return node;
}

boost::shared_ptr<Node> Node::getParent(){
    if(boost::shared_ptr<Node> parent = m_parent.lock())
    {
        return parent;
    } else throw std::out_of_range("Node has no parent");
}
