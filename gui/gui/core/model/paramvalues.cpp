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

#include "paramvalues.h"

using namespace cauv;
using namespace cauv::gui;

#include "nodes/numericnode.h"
#include "nodes/stringnode.h"


ParamValueToNode::ParamValueToNode(const nid_t id) :
    m_id(id) {
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(int & operand) const
{
    boost::shared_ptr<NumericNode<int> > node = boost::make_shared<NumericNode<int> >(m_id);
    node->update(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(float & operand) const
{
    boost::shared_ptr<NumericNode<float> > node = boost::make_shared<NumericNode<float> >(m_id);
    node->update(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(std::string & operand) const
{
    boost::shared_ptr<StringNode> node = boost::make_shared<StringNode>(m_id);
    node->update(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(bool & operand) const
{
    boost::shared_ptr<NumericNode<bool> > node = boost::make_shared<NumericNode<bool> >(m_id);
    node->update(operand);
    return node;
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(BoundedFloat & operand) const
{
    boost::shared_ptr<NumericNode<BoundedFloat> > node = boost::make_shared<NumericNode<BoundedFloat> >(m_id);
    node->update(operand);
    return node;
}
