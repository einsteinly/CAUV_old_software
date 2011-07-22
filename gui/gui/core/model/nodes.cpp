#include "model.h"

using namespace cauv;
using namespace cauv::gui;


NodeBase::NodeBase(GuiNodeType::e t, const id_variant_t id) :
        type(t), m_parent(), m_id(id), m_mutable(false) {

    qRegisterMetaType<boost::shared_ptr<NodeBase> >("boost::shared_ptr<NodeBase>");
}

NodeBase::~NodeBase(){
    debug(0) << "~NodeBase" << nodeName();
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

void NodeBase::addChild(boost::shared_ptr<NodeBase> child){
    if(boost::shared_ptr<NodeBase> parent = child->m_parent.lock())
    {
        std::stringstream str;
        str << "Node already has a parent: " << parent->nodeName();
        throw std::runtime_error(str.str());
    }

    child->m_parent = boost::weak_ptr<NodeBase>(shared_from_this());
    m_children.push_back(child);

    // propagate changes upwards
    child->connect(child.get(), SIGNAL(changed()), this, SIGNAL(changed()));

    Q_EMIT nodeAdded(child);
}

const std::vector<boost::shared_ptr<NodeBase> > NodeBase::getChildren() const {
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
