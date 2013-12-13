#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <map>
#include <string>

#include <XmlRpcValue.h>

namespace cauv {
namespace pipeline_model {

enum class ParamLinkType {
    Input,
    Output,
};

class NodeModel;

class ParamValue {
    public:
    virtual XmlRpc::XmlRpcValue toXmlRpcValue() = 0;
    virtual std::string getType() = 0;
    virtual boost::shared_ptr<ParamValue> clone() = 0;
    virtual bool canAccept(ParamValue &other) {
        return getType() == other.getType();
    }
    virtual ~ParamValue() {};
    protected:
    ParamValue() {};
};

//little bit of magic to produce a registry of subclasses of ParamValue
//see http://stackoverflow.com/questions/10332725/how-to-automatically-register-a-class-on-creation
class ParamTypeRegistry {
    public:
    typedef boost::shared_ptr<ParamValue> (*TypeConstructor)(XmlRpc::XmlRpcValue&);
    static boost::shared_ptr<ParamValue> getParamValue(std::string type, XmlRpc::XmlRpcValue&);
    static void addParamType(std::string type, TypeConstructor constructor);
    private:
    static std::map<std::string, TypeConstructor> &registry();
};

class NoSuchParamTypeException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

template<class ParamValueType>
class ParamValueReg : public ParamValue {
    public:
    ParamValueReg() {
        (void)reg;
    }
    static bool registerValueType() {
        ParamValueType t;
        std::cout << "registering C++ type " << typeid(t).name() << " as Param type " << t.getType() << std::endl;
        ParamTypeRegistry::addParamType(t.getType(), &ParamValueType::fromXmlRpcValue);
        return true;
    }
    virtual boost::shared_ptr<ParamValue> clone() {
        return boost::make_shared<ParamValueType>(*dynamic_cast<ParamValueType*>(this));
    }
    private:
    static bool reg;
};

template <class ParamValueType>
bool ParamValueReg<ParamValueType>::reg = ParamValueReg<ParamValueType>::registerValueType();

//model for parameter
class ParamModel {
    public:
    ParamModel(const std::string name,
               const std::string description,
               const ParamLinkType link_type,
               const boost::shared_ptr<ParamValue> value);
    const std::string name;
    const std::string description; // Used for e.g. mouseover
    const ParamLinkType link_type; // Is this an input/output parameter
    const boost::shared_ptr<ParamValue> value;

    std::string getType() { return value->getType(); }
};

//model for parameter bound to a node
class BoundParamModel : public ParamModel {
    public:
    BoundParamModel(const ParamModel &model, const boost::shared_ptr<NodeModel> node);
    const boost::shared_ptr<NodeModel> node;
};

//Possible ParamValue types (uses magic above to autoregister types on creation)
class FloatParam: public ParamValueReg<FloatParam> {
    public:
    FloatParam() : value(0) {}
    FloatParam(float value_) : value(value_) {}
    float value;
    virtual std::string getType() { return "float"; }
    virtual XmlRpc::XmlRpcValue toXmlRpcValue() { return XmlRpc::XmlRpcValue(value); }
    static boost::shared_ptr<ParamValue> fromXmlRpcValue(XmlRpc::XmlRpcValue &val) { 
        return boost::make_shared<FloatParam>(static_cast<double>(val));
    }
};

class BoolParam: public ParamValueReg<BoolParam> {
    public:
    BoolParam() : value(0) {}
    BoolParam(bool value_) : value(value_) {}
    bool value;
    virtual std::string getType() { return "bool"; }
    virtual XmlRpc::XmlRpcValue toXmlRpcValue() { return XmlRpc::XmlRpcValue(value); }
    static boost::shared_ptr<ParamValue> fromXmlRpcValue(XmlRpc::XmlRpcValue &val) {
        return boost::make_shared<BoolParam>(val);
    }
};

class IntParam: public ParamValueReg<IntParam> {
    public:
    IntParam() : value(0) {}
    IntParam(int value_) : value(value_) {}
    int value;
    virtual std::string getType() { return "int"; }
    virtual XmlRpc::XmlRpcValue toXmlRpcValue() { return XmlRpc::XmlRpcValue(value); }
    static boost::shared_ptr<ParamValue> fromXmlRpcValue(XmlRpc::XmlRpcValue& val) { 
        return boost::make_shared<IntParam>(val);
    }
};

}
}
