#include "param_model.h"

#include <XmlRpcValue.h>
#include <stdexcept>

using namespace cauv;
using namespace pipeline_model;

ParamModel::ParamModel(const std::string name_,
           const std::string description_,
           const ParamLinkType link_type_,
           const boost::shared_ptr<ParamValue> value_) :
    name(name_),
    description(description_),
    link_type(link_type_),
    value(value_)
{

}

//Static initialisation fiasco avoidance.
std::map<std::string, ParamTypeRegistry::TypeConstructor> &ParamTypeRegistry::registry() {
    static std::map<std::string, ParamTypeRegistry::TypeConstructor> registry_map;
    return registry_map;
};

boost::shared_ptr<ParamValue>
ParamTypeRegistry::getParamValue(std::string type, XmlRpc::XmlRpcValue &t) {
    if (registry().count(type)) {
        return (registry()[type])(t);
    } else {
        throw NoSuchParamTypeException("Tried to create ParamValue of type " + type + " But don't have a constructor");
    }
    return boost::shared_ptr<ParamValue>();
}

void ParamTypeRegistry::addParamType(std::string type, ParamTypeRegistry::TypeConstructor constructor) {
    registry()[type] = constructor;
}

BoundParamModel::BoundParamModel(const ParamModel &model, boost::shared_ptr<NodeModel> node_) :
    ParamModel(model.name, model.description, model.link_type, model.value->clone()),
    node(node_) {

}
