/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "paramvalues.h"

using namespace cauv;
using namespace cauv::gui;

#include "nodes/numericnode.h"
#include "nodes/stringnode.h"
#include "nodes/colournode.h"


ParamValueToNode::ParamValueToNode(const nid_t id) :
    m_id(id) {
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(int & operand) const
{
    boost::shared_ptr<NumericNode<int> > node = boost::make_shared<NumericNode<int> >(m_id);
    node->typedUpdate(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(float & operand) const
{
    boost::shared_ptr<NumericNode<float> > node = boost::make_shared<NumericNode<float> >(m_id);
    node->typedUpdate(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(const std::string& operand) const
{
    boost::shared_ptr<StringNode> node = boost::make_shared<StringNode>(m_id);
    node->update(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(bool & operand) const
{
    boost::shared_ptr<BooleanNode> node = boost::make_shared<BooleanNode>(m_id);
    node->typedUpdate(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(BoundedFloat & operand) const
{
    boost::shared_ptr<NumericNode<BoundedFloat> > node = boost::make_shared<NumericNode<BoundedFloat> >(m_id);
    node->typedUpdate(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(Colour & operand) const
{
    boost::shared_ptr<ColourNode> node = boost::make_shared<ColourNode>(m_id);
    node->typedUpdate(operand);
    return node;
}

boost::shared_ptr<Node> cauv::gui::paramWithMetaToNode(nid_t id, ParamWithMeta & param_with_meta)
{
    boost::shared_ptr<Node> node = paramValueToNode(id, param_with_meta.value);
    node->setDocstring(param_with_meta.docstring);
    try {
        boost::shared_ptr<NumericNodeBase> num_node = node->to<NumericNodeBase>();
        num_node->setUnits(param_with_meta.units);
    }
    catch(std::runtime_error& e){
        //can't set units, as isnt numeric node
    }
    return node;
}
