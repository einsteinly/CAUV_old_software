#include "node_model.h"
#include "pipeline.h"
#include <algorithm>
#include <sstream>

using namespace cauv;


std::map<std::string, const NodeModelType> NodeModelType::node_types;

int NodeModel::current_id = 1;

NodeModelType::NodeModelType() :
    name("null") {

}

NodeModelType::NodeModelType(const std::string type) :
    name(type) {
        
}

void NodeModelType::addOutput(const std::string name, 
                          const std::string description,
                          ParamValue &default_value) {
    outputs.emplace(name, ParamModel(name, description, ParamLinkType::Output, default_value.clone()));
}

void NodeModelType::addInput(const std::string name, 
                          const std::string description,
                          ParamValue &default_value) {
    inputs.emplace(name, ParamModel(name, description, ParamLinkType::Input, default_value.clone()));
}

void NodeModelType::addType(const NodeModelType &type) {
    node_types.insert(std::make_pair(type.name, type));
}

NodeModel::NodeModel(const std::string type_, PipelineModel &pipeline_) :
 type(NodeModelType::node_types.at(type_)),
 id(current_id),
 pipeline(pipeline_) {
     current_id++;
     std::stringstream name_str;
     name_str << type.name << " " << id;
     setName(name_str.str());
}

void NodeModel::setName(const std::string &new_name) {
    if (name == new_name) {
        return;
    }
    if (pipeline.node_names.count(new_name)) {
        throw DuplicateNodeNameException("Tried to assign name " + new_name + " to node but it already exists");
    }
    pipeline.node_names.erase(name);
    name = new_name;
    pipeline.node_names.emplace(name);
}

void NodeModel::connectOutput(const std::string output_name, InputModel& input) {
    auto &output = getOutput(output_name);
    if (!input.value->canAccept(*output.value)) {
        throw IncompatibleTypesException("Input of type " + input.value->getType() + " cannot accept output of type " + output.value->getType());
    }
    output.outputs.push_back(input);
    input.input = &output;
}

void NodeModel::connectInput(const std::string input_name, OutputModel& output) {
    //TODO
}

void NodeModel::disconnectInput(const std::string input_name) {
    if (!inputs.count(input_name)) {
        return;
    }
    auto &input = getInput(input_name);
    if (input.input) {
        auto &outputs = input.input->outputs;
        auto end_it = std::remove_if(outputs.begin(), outputs.end(),
                [&input](std::reference_wrapper<InputModel> &link) {
                return &input == &link.get();
            });
        outputs.erase(end_it, outputs.end());
        input.input = nullptr;
    }
}

void NodeModel::disconnectOutput(const std::string output_name) {
    if (! outputs.count(output_name)) {
        return;
    }
    auto &output = getOutput(output_name);
    for (auto &link : output.outputs) {
        link.get().input = nullptr;
    }
    output.outputs.clear();
}

void NodeModel::isolate() {
    for (auto &output: outputs) {
        disconnectOutput(output.first);
    }
    for (auto &input: inputs) {
        disconnectInput(input.first);
    }
}

NodeModel::~NodeModel() {
    isolate();
    pipeline.node_names.erase(name);
}
