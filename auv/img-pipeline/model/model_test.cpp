#define BOOST_TEST_MAIN
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#include <iostream>
#include "pipeline.h"

using namespace cauv;

BOOST_AUTO_TEST_CASE( paramValueFloatCreate )  {
    auto val = XmlRpc::XmlRpcValue(3.0);
    auto paramValue = ParamTypeRegistry::getParamValue("float", val);
    BOOST_CHECK(paramValue);
}

BOOST_AUTO_TEST_CASE( paramValueBoolCreate )  {
    auto val = XmlRpc::XmlRpcValue(true);
    auto paramValue = ParamTypeRegistry::getParamValue("bool", val);
    BOOST_CHECK(paramValue);
}

BOOST_AUTO_TEST_CASE( paramValueIntCreate )  {
    auto val = XmlRpc::XmlRpcValue(3);
    auto paramValue = ParamTypeRegistry::getParamValue("int", val);
    BOOST_CHECK(paramValue);
}

BOOST_AUTO_TEST_CASE( nonExistantCreate ) {
    auto val = XmlRpc::XmlRpcValue(3);
    BOOST_CHECK_THROW(ParamTypeRegistry::getParamValue("_doesntexist", val),
                      NoSuchParamTypeException);
}

BOOST_AUTO_TEST_CASE ( paramValueCheck ) {
    auto val = XmlRpc::XmlRpcValue(3.0);
    auto paramValue = ParamTypeRegistry::getParamValue("float", val);
    BOOST_CHECK(paramValue);
    auto floatValue = boost::dynamic_pointer_cast<FloatParam>(paramValue);
    BOOST_CHECK(floatValue);
    BOOST_CHECK_EQUAL(floatValue->value, 3.0);
}

BOOST_AUTO_TEST_CASE ( toXmlRpcCheck ) {
    auto val = XmlRpc::XmlRpcValue(3.0);
    auto paramValue = ParamTypeRegistry::getParamValue("float", val);
    BOOST_CHECK(paramValue);
    BOOST_CHECK(paramValue->toXmlRpcValue() == val);
}

BOOST_AUTO_TEST_CASE ( cloneValue ) {
    boost::shared_ptr<ParamValue> value = boost::make_shared<FloatParam>(4.0);
    auto value_2 = value->clone();
    BOOST_CHECK(value_2);
    auto value_3 = boost::dynamic_pointer_cast<FloatParam>(value_2);
    BOOST_CHECK_EQUAL(value_3->value, 4.0);
}

BOOST_AUTO_TEST_CASE ( nodeType ) {
    auto new_type = NodeModelType("test_type");
    auto value = FloatParam(4.0);
    new_type.addOutput("value", "test value", value);
    new_type.addInput("value", "test value", value);
    NodeModelType::addType(new_type);
    PipelineModel pipeline;

    auto new_node = pipeline.addNode("test_type");
    BOOST_CHECK_EQUAL(new_node->type.name, "test_type");
    auto output = new_node->getOutput("value");
    auto output_value = boost::dynamic_pointer_cast<FloatParam>(output.value);
    BOOST_CHECK(output_value);
    BOOST_CHECK_EQUAL(output_value->value, 4.0);

    auto input = new_node->getInput("value");
}

BOOST_AUTO_TEST_CASE ( connectInput ) {
    auto new_type = NodeModelType("connect_test_type");
    auto value = FloatParam(4.0);
    new_type.addOutput("value", "test value", value);
    new_type.addInput("value", "test value", value);
    NodeModelType::addType(new_type);
    PipelineModel pipeline;

    auto source_node = pipeline.addNode("connect_test_type");
    auto sink_node = pipeline.addNode("connect_test_type");

    source_node->connectOutput("value", sink_node->getInput("value"));

    BOOST_CHECK_EQUAL(source_node->getOutput("value").outputs.size(), 1);
    BOOST_CHECK_EQUAL(&source_node->getOutput("value").outputs[0].get(), &sink_node->getInput("value"));
    BOOST_CHECK(sink_node->getInput("value").input);
    BOOST_CHECK_EQUAL(sink_node->getInput("value").input->node, source_node);
}

BOOST_AUTO_TEST_CASE ( invalidConnect ) {
    auto new_type = NodeModelType("invalid_connect_test_type");
    auto value = FloatParam(4.0);
    new_type.addOutput("value", "test value", value);
    auto value_2 = IntParam(4);
    new_type.addInput("value", "test value", value_2);
    NodeModelType::addType(new_type);
    PipelineModel pipeline;

    auto source_node = pipeline.addNode("invalid_connect_test_type");
    auto sink_node = pipeline.addNode("invalid_connect_test_type");

    BOOST_CHECK_THROW(source_node->connectOutput("value", sink_node->getInput("value")),
                      IncompatibleTypesException);

    BOOST_CHECK_THROW(source_node->getOutput("nonexistant"),
                      NoSuchParamException);
    BOOST_CHECK_THROW(source_node->getInput("nonexistant"),
                      NoSuchParamException);
}

BOOST_AUTO_TEST_CASE ( pipelineToXml ) {
    auto new_type = NodeModelType("xml_test_type");
    auto float_param = FloatParam(4.0);
    auto int_param = IntParam(4);
    new_type.addOutput("float_output", "test value", float_param);
    new_type.addOutput("int_output", "test value", int_param);
    new_type.addInput("float_input", "test value", float_param);
    new_type.addInput("int_input", "test value", int_param);
    NodeModelType::addType(new_type);

    PipelineModel pipeline;

    auto source_node = pipeline.addNode("xml_test_type");
    auto sink_node = pipeline.addNode("xml_test_type");

    source_node->connectOutput("float_output", sink_node->getInput("float_input"));

    XmlRpc::XmlRpcValue xml_pipeline = pipeline.toXmlRpcValue();

    //std::cout << xml_pipeline.toXml();

    PipelineModel pipeline2;

    pipeline2.updateFromXmlRpcValue(xml_pipeline);
}

BOOST_AUTO_TEST_CASE ( disconnection ) {
    PipelineModel pipeline;
    auto source_node = pipeline.addNode("connect_test_type");
    auto sink_node = pipeline.addNode("connect_test_type");

    source_node->connectOutput("value", sink_node->getInput("value"));

    source_node->disconnectOutput("value");
    BOOST_CHECK_EQUAL(source_node->getOutput("value").outputs.size(), 0);
    //nullptr would be nicer, but you can't do std::cout << nullptr so it
    //won't work
    BOOST_CHECK_EQUAL(sink_node->getInput("value").input, (void*)NULL);

    source_node->disconnectOutput("nonexistant");
    source_node->disconnectInput("nonexistant");

    source_node->connectOutput("value", sink_node->getInput("value"));
    sink_node->disconnectInput("value");
    BOOST_CHECK_EQUAL(source_node->getOutput("value").outputs.size(), 0);
    BOOST_CHECK_EQUAL(sink_node->getInput("value").input, (void*)NULL);

    source_node->connectOutput("value", sink_node->getInput("value"));
    source_node->isolate();
    BOOST_CHECK_EQUAL(source_node->getOutput("value").outputs.size(), 0);
    BOOST_CHECK_EQUAL(sink_node->getInput("value").input, (void*)NULL);

    source_node->connectOutput("value", sink_node->getInput("value"));
    sink_node->isolate();
    BOOST_CHECK_EQUAL(source_node->getOutput("value").outputs.size(), 0);
    BOOST_CHECK_EQUAL(sink_node->getInput("value").input, (void*)NULL);
}

BOOST_AUTO_TEST_CASE ( deleteNode ) {
    PipelineModel pipeline;
}

BOOST_AUTO_TEST_CASE ( duplicateNodeNames ) {
    PipelineModel pipeline;
    NodeModelType::addType(NodeModelType("duplicate_node_name_test_type"));
    auto node_1 = pipeline.addNode("duplicate_node_name_test_type");
    auto node_2 = pipeline.addNode("duplicate_node_name_test_type");
    node_1->setName("duplicate_name");
    BOOST_CHECK_EQUAL(node_1->getName(), "duplicate_name");
    BOOST_CHECK_THROW(node_2->setName("duplicate_name"), DuplicateNodeNameException);
    //Dangling shared pointers to the node might still exist, but it is
    //conceptually isolated from the others nodes at this point.
    pipeline.delNode("duplicate_name");
    node_2->setName("duplicate_name");
}

#if 0
int main() {
    PipelineModel m;
    auto val = XmlRpc::XmlRpcValue(3.0);
    auto param = ParamTypeRegistry::getParamValue("float", val);
    assert(param);
    auto val2 = boost::dynamic_pointer_cast<FloatParam>(param);
    std::cout << val2->value << std::endl;
    std::cout << param->toXmlRpcValue().toXml() << std::endl;
    std::cout << "ASDF" << std::endl;
}
#endif
