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


NodeBase::NodeBase(GuiNodeType::e t, id_variant_t const& id) :
        type(t), m_parent(), m_id(id), m_mutable(false) {

    qRegisterMetaType<boost::shared_ptr<NodeBase> >("boost::shared_ptr<NodeBase>");
}

NodeBase::~NodeBase(){
    debug(2) << "~NodeBase" << nodeName();
}

std::string NodeBase::nodeName() const {
    return boost::apply_visitor(id_to_name(), m_id);
}

id_variant_t NodeBase::nodeId() const {
    return m_id;
}

std::string NodeBase::nodePath() const {
    std::stringstream stream;

    if(!m_parent.expired()) {
        boost::shared_ptr<NodeBase> parent = m_parent.lock();
        if(parent)
            stream << parent->nodePath() << "/" << nodeName();
    }
    else stream << nodeName();

    return stream.str();
}

void NodeBase::addChild(boost::shared_ptr<NodeBase> const& child){
    if(boost::shared_ptr<NodeBase> parent = child->m_parent.lock())
    {
        std::stringstream str;
        str << "Node already has a parent: " << parent->nodeName();
        throw std::runtime_error(str.str());
    }

    child->m_parent = boost::weak_ptr<NodeBase>(shared_from_this());
    m_children.push_back(child);
    m_id_map[child->nodeId()] = child;

    // make sure all the objects belong to the thread of the root item
    // children get created in many threads (e.g. the messaging thread)
    // and these threads don't have event loops so signals won't work
    child->moveToThread(this->thread());

    // propagate changes upwards
    child->connect(child.get(), SIGNAL(changed()), this, SIGNAL(changed()));
    child->connect(child.get(), SIGNAL(treeChanged()), this, SIGNAL(treeChanged()));

    Q_EMIT nodeAdded(child);
    Q_EMIT treeChanged();
}

const NodeBase::children_list_t NodeBase::getChildren() const {
    return m_children;
}

bool NodeBase::isMutable() const{
    return m_mutable;
}

void NodeBase::setMutable(bool mut){
    m_mutable = mut;
}

boost::shared_ptr<NodeBase> NodeBase::getRoot() {
    boost::shared_ptr<NodeBase> node = shared_from_this();

    while(node->m_parent.lock())
        node = node->m_parent.lock();

    return node;
}
