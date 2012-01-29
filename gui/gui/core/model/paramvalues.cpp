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


ParamValueToNode::ParamValueToNode(const nid_t id, boost::shared_ptr<Node> parent) :
    m_id(id), m_parent(parent){
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(int &) const
{
    return m_parent->findOrCreate<NumericNode<int> >(m_id);
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(float & ) const
{
    return m_parent->findOrCreate<NumericNode<float> >(m_id);
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(std::string & ) const
{
    return m_parent->findOrCreate<StringNode>(m_id);
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(bool & operand ) const
{
    return m_parent->findOrCreate<NumericNode<bool> >(m_id);
}

template <> boost::shared_ptr<Node> ParamValueToNode::operator()(BoundedFloat & ) const
{
    return m_parent->findOrCreate<NumericNode<BoundedFloat> >(m_id);
}



template <> QVariant ParamValueToQVariant::operator()(std::string & s) const {
    error() << "input value: " << s;
    QVariant v = QVariant::fromValue(QString::fromStdString(s));
    error() << "valid? " << v.isValid();
    error() << "variant value: " << v.toString().toStdString();
    return v;
}

