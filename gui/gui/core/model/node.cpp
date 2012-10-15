/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */
#include "node.h"

#include <QMetaType>
#include <QUrl>

using namespace cauv;
using namespace cauv::gui;


int node_types::count = 0;
std::map<std::string, int> node_types::typeMap;


Node::Node(nid_t const& id, node_type t) :
    type(t), m_parent(), m_id(id), m_mutable(false) {

    qRegisterMetaType<boost::shared_ptr<Node> >("boost::shared_ptr<Node>");
}

Node::~Node(){
    debug(2) << "~Node" << nodeName();
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
    {
        lock_t l(m_childrenLock);

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
    }
    // make sure all the objects belong to the thread of the root item
    // children get created in many threads (e.g. the messaging thread)
    // and these threads don't have event loops so signals won't work
    child->moveToThread(this->thread());

    // propagate some changes upwards
    child->connect(child.get(), SIGNAL(onBranchChanged()), this, SIGNAL(onBranchChanged()));
    child->connect(child.get(), SIGNAL(structureChanged()), this, SIGNAL(structureChanged()));

    // propagate add/removes downwards
    connect(this, SIGNAL(detachedFrom(boost::shared_ptr<Node>)),
            child.get(), SIGNAL(detachedFrom(boost::shared_ptr<Node>)));
    connect(this, SIGNAL(attachedTo(boost::shared_ptr<Node>)),
            child.get(), SIGNAL(attachedTo(boost::shared_ptr<Node>)));

    Q_EMIT childAdded(child);
    Q_EMIT child->attachedTo(shared_from_this());
    Q_EMIT structureChanged();
}


bool Node::removeChild(boost::shared_ptr<Node> const& child){
    
    lock_t l(m_creationLock);
    
    //check it's actually a child of this node
    try {
        find<Node>(child->nodeId());
    } catch (std::out_of_range){
        return false;
    }

    Q_EMIT childRemoved(child);
    Q_EMIT child->detachedFrom(shared_from_this());
    Q_EMIT structureChanged();

    // disconnect propagation signals
    child->disconnect(child.get(), SIGNAL(onBranchChanged()), this, SIGNAL(onBranchChanged()));
    child->disconnect(child.get(), SIGNAL(structureChanged()), this, SIGNAL(structureChanged()));

    // propagate removes downwards
    child->disconnect(this, SIGNAL(detachedFrom(boost::shared_ptr<Node>)),
                     child.get(), SIGNAL(detachedFrom(boost::shared_ptr<Node>)));
    child->disconnect(this, SIGNAL(attachedTo(boost::shared_ptr<Node>)),
                     child.get(), SIGNAL(attachedTo(boost::shared_ptr<Node>)));

    {
        lock_t l(m_childrenLock);
        // clear parent from child
        child->m_parent.reset();
        m_children.erase(std::find(m_children.begin(), m_children.end(), child));
        m_id_map.erase(child->nodeId());
        m_id_map.erase(child->nodeName());
    }

    return true;
}

bool Node::removeChild(nid_t const& childId){
    return removeChild(find<Node>(childId));
}

const Node::children_list_t Node::getChildren() const {
    lock_t l(m_childrenLock);
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

boost::shared_ptr<Node> Node::getParent() {
    if(boost::shared_ptr<Node> parent = m_parent.lock())
    {
        return parent;
    } else throw std::out_of_range("Node has no parent");
}

int Node::row() const{
    lock_t l(m_childrenLock);
    // work out the row of the parent with respect to its siblings
    boost::shared_ptr<const Node> parent = m_parent.lock();
    if(!parent) return 0; // root node
    int row = 0;
    foreach (boost::shared_ptr<Node> const& c, parent->getChildren()){
        if (c.get() == this) return row;
        row++;
    }
    return row;
}

int Node::columnCount() const{
    return 1;
}

void Node::update(QVariant const& value){
    //if(m_value.userType() != value.userType()) {
    //    error() << "Node::update() Type mismatch in Node variant:" << nodePath();
    //}

    // !!! if this check causes problems (I don't think it should), then don't
    // remove it without adding this check in at a higher level for fluidity
    // node inputs: most time spent rendering is due to changed parameters!
    if(m_value != value){
        m_value = value;
        Q_EMIT onUpdate(value);
        Q_EMIT onUpdate();
        debug(8) << nodePath() << "updated to " << value.toString().toStdString() << "type:" << value.typeName();
    }else{
        debug(8) << nodePath() << "remains at " << value.toString().toStdString() << "type:" << value.typeName();
    }
}

bool Node::set(QVariant const& value){
    //if(m_value.userType() != value.userType()) {
    //    error() << "Node::set() Type mismatch in Node variant:" << nodePath();
    // }
    debug(2) << nodePath() << "set to" << value.toString().toStdString() << "type:" << value.typeName();
    update(value);
    Q_EMIT onSet(value);
    Q_EMIT onSet();
    Q_EMIT onBranchChanged();
    return true;
}

const QVariant Node::get() const{
    return m_value;
}
