/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <boost/shared_ptr.hpp>

#include "paramvalues.h"

using namespace cauv;
using namespace cauv::gui;
using namespace cauv::pipeline_model;

#include "nodes/numericnode.h"
#include "nodes/stringnode.h"
#include "nodes/colournode.h"

boost::shared_ptr<Node> IntParamValueToNode(std::string const& name, boost::shared_ptr<ParamValue> pv)
{
    try {
        auto ipv = boost::dynamic_pointer_cast<IntParam>(pv);
        boost::shared_ptr<NumericNode<int> > node = boost::make_shared<NumericNode<int> >(name);
        node->typedUpdate(ipv->value);
        return node;
    } catch (std::bad_cast) {
        throw std::runtime_error("Could not cast ParamValue of type int to IntParam");
    }
}

boost::shared_ptr<Node> FloatParamValueToNode(std::string const& name, boost::shared_ptr<ParamValue> pv)
{
    try {
        auto fpv = boost::dynamic_pointer_cast<FloatParam>(pv);
        boost::shared_ptr<NumericNode<float> > node = boost::make_shared<NumericNode<float> >(name);
        node->typedUpdate(fpv->value);
        return node;
    } catch (std::bad_cast) {
        throw std::runtime_error("Could not cast ParamValue of type float to FloatParam");
    }
}

//TODO implement string param values
// boost::shared_ptr<Node> ParamValueToNode()(StringParamValue& pv) const
// {
//     boost::shared_ptr<StringNode> node = boost::make_shared<StringNode>(m_id);
//     node->update(operand);
//     return node;
// }

boost::shared_ptr<Node> BoolParamValueToNode(std::string const& name, boost::shared_ptr<ParamValue> pv)
{
    try {
        auto bpv = boost::dynamic_pointer_cast<BoolParam>(pv);
        boost::shared_ptr<BooleanNode> node = boost::make_shared<BooleanNode>(name);
        node->typedUpdate(bpv->value);
        return node;
    } catch (std::bad_cast) {
        throw std::runtime_error("Could not cast ParamValue of type bool to BoolParam");
    }
}

#if 0
//TODO impement bounded and colour param values
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
#endif



boost::shared_ptr<Node> paramModelToNode(ParamModel& pm){
    static std::map<std::string,
                    std::function< boost::shared_ptr<Node>(std::string const&,
                                                           boost::shared_ptr<ParamValue>)
                                 >
                   >dispatcher = {
        {"int", IntParamValueToNode},
        {"float", FloatParamValueToNode},
        {"bool", BoolParamValueToNode},
    };
    try {
        dispatcher.at(pm.getType())(pm.name, pm.value);
    } catch (std::out_of_range){
        throw std::runtime_error("Unsupported ParamValue type");
    }
}

#if 0
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
#endif
