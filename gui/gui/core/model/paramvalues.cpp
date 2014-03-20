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

//ensures that required methods are implemented
struct ConverterBase
{
    virtual boost::shared_ptr<Node> toNode(std::string const& name) = 0;
    virtual QVariant toQVariant() = 0;
};

template< typename T >
struct Converter : ConverterBase
{
    Converter(boost::shared_ptr<ParamValue> pv) : ConverterBase() {
        m_pv = boost::dynamic_pointer_cast< T >(pv);
        if (!m_pv){ throw std::runtime_error("Could not cast ParamValue"); }
    }
    virtual QVariant toQVariant(){
        return QVariant(m_pv->value);
    }
    virtual boost::shared_ptr<Node> toNode(std::string const& name);
private:
    boost::shared_ptr< T > m_pv;
};

//TODO string specialisation
// template<>
// QVariant Converter<StringParam>::toQVariant(){
//     return QString::fromStdString(m_pv->value())
// }

template<>
boost::shared_ptr<Node> Converter<IntParam>::toNode(std::string const& name){
    boost::shared_ptr<NumericNode<int> > node = boost::make_shared<NumericNode<int> >(name);
    node->typedUpdate(m_pv->value);
    return node;
}

template<>
boost::shared_ptr<Node> Converter<FloatParam>::toNode(std::string const& name)
{
    boost::shared_ptr<NumericNode<float> > node = boost::make_shared<NumericNode<float> >(name);
    node->typedUpdate(m_pv->value);
    return node;
}

template<>
boost::shared_ptr<Node> Converter<BoolParam>::toNode(std::string const& name)
{
    boost::shared_ptr<BooleanNode> node = boost::make_shared<BooleanNode>(name);
    node->typedUpdate(m_pv->value);
    return node;
}

//TODO implement string param values
// boost::shared_ptr<Node> ParamValueToNode()(StringParamValue& pv) const
// {
//     boost::shared_ptr<StringNode> node = boost::make_shared<StringNode>(m_id);
//     node->update(operand);
//     return node;
// }

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

//"constructor" that can be stored
template< typename T>
boost::shared_ptr<ConverterBase> constructConverter(boost::shared_ptr<ParamValue> pv){
    return boost::make_shared<Converter<T>>(pv);
}

boost::shared_ptr<ConverterBase> getConverter(boost::shared_ptr<ParamValue> pv){
    static std::map<std::string,
                    std::function< boost::shared_ptr<ConverterBase>(boost::shared_ptr<ParamValue>)>
                   >dispatcher = {
        {"int", constructConverter<IntParam>},
        {"float", constructConverter<FloatParam>},
        {"bool", constructConverter<BoolParam>},
    };
    try {
        return dispatcher.at(pv->getType())(pv);
    } catch (std::out_of_range) {
        throw std::runtime_error("Unsupported ParamValue");
    }
}


namespace cauv{
namespace gui{
boost::shared_ptr<Node> paramModelToNode(ParamModel& pm){
    return paramValueToNode(pm.name, pm.value);
}

boost::shared_ptr<Node> paramValueToNode(std::string const& id, boost::shared_ptr<ParamValue> pv){
    return getConverter(pv)->toNode(id);
}

QVariant paramValueToQVariant(boost::shared_ptr<ParamValue> pv){
    return getConverter(pv)->toQVariant();
}
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
