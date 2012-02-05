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

#include "conditionnode.h"

#include <debug/cauv_debug.h>

#include <gui/core/model/paramvalues.h>

using namespace cauv;
using namespace cauv::gui;


std::set<std::string> AiConditionNode::m_types;


AiConditionNode::AiConditionNode(const nid_t id) : Node(id, nodeType<AiConditionNode>()){
}

boost::shared_ptr<Node> AiConditionNode::setDebug(std::string name, ParamValue value){
    if (!m_debug[name]) {
        m_debug[name] = paramValueToNode(nid_t(name), value);
        addChild(m_debug[name]);
    }
    m_debug[name]->update(variantToQVariant(value));
    return m_debug[name];
}

void AiConditionNode::removeDebug(std::string name){
    this->removeChild(nid_t(name));
    m_debug.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiConditionNode::getDebugValues(){
    return m_debug;
}

boost::shared_ptr<Node> AiConditionNode::setOption(std::string name, ParamValue value){
    if (!m_options[name]) {
        m_options[name] = paramValueToNode(nid_t(name), value);
        addChild(m_options[name]);
    }
    m_options[name]->update(variantToQVariant(value));
    return m_options[name];
}

void AiConditionNode::removeOption(std::string name){
    this->removeChild(nid_t(name));
    m_options.erase(name);
}

std::map<std::string, boost::shared_ptr<Node> > AiConditionNode::getOptions(){
    return m_options;
}


void AiConditionNode::addType(std::string type){
    m_types.insert(type);
}

std::set<std::string> AiConditionNode::getTypes(){
    return m_types;
}
