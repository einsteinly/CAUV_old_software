#include "model.h"

using namespace cauv;
using namespace cauv::gui;


NodeBase::NodeBase(GuiNodeType::e t, const std::string name) :
        type(t), m_parent(), m_name(name), m_mutable(false) {

    qRegisterMetaType<boost::shared_ptr<NodeBase> >("boost::shared_ptr<NodeBase>");
}

NodeBase::~NodeBase(){
    debug(0) << "~NodeBase" << nodeName();
}

std::string NodeBase::nodeName(const bool full) {
    std::stringstream stream;

    if(full && !m_parent.expired()) {
        boost::shared_ptr<NodeBase> parent = m_parent.lock();
        if(parent)
            stream << parent->nodeName() << "/" << m_name;
    }
    else stream << m_name;

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
    connect(child.get(), SIGNAL(changed()), this, SIGNAL(changed()));

    Q_EMIT nodeAdded(child);
}

const std::vector<boost::shared_ptr<NodeBase> > NodeBase::getChildren() const {
    return m_children;
}

bool NodeBase::isMutable(){
    return m_mutable;
}

void NodeBase::setMutable(bool mut){
    m_mutable = mut;
}
