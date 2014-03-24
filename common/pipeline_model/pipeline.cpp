#include "pipeline.h"
#include <boost/make_shared.hpp>
#include <algorithm>
#include <sstream>

using namespace cauv;
using namespace pipeline_model;

/*
 *--------------Constructors/Destructors
 */

PipelineModel::PipelineModel(const std::string& pipeline_name) : pipeline_name(pipeline_name){}

/*
 * -------------Access information
 */

const std::string& PipelineModel::pipelineName(){
    return pipeline_name;
}

/*
 * -------------Node Manipulation
 */


boost::shared_ptr<NodeModel> PipelineModel::constructNode(const std::string type) {
    return boost::make_shared<NodeModel>(type, *this);
}

boost::shared_ptr<NodeModel> PipelineModel::addNode(const std::string type) {
    auto new_node = constructNode(type);
    nodes.push_back(new_node);
    return new_node;
}

void PipelineModel::delNode(const std::string &name) {
    //removes by shifting everything down, and returns new end point
    auto end_it = std::remove_if(nodes.begin(), nodes.end(),
                                 [&name](boost::shared_ptr<NodeModel> &n) {
                                   if (n->getName() == name) {
                                     n->isolate();
                                     std::stringstream deleted_name;
                                     deleted_name << "__deleted " << n->id;
                                     n->setName(deleted_name.str());
                                     return true;
                                   } else {
                                     return false;
                                   } 
                                });
    //delete everything past the new end point
    nodes.erase(end_it, nodes.end());
}

// void PipelineModel::delNodeById(NodeId id) {
//     auto end_it = std::remove_if(nodes.begin(), nodes.end(),
//                                  [&id](boost::shared_ptr<NodeModel> &n) {
//                                    if (n->id == id) {
//                                      n->isolate();
//                                      std::stringstream deleted_name;
//                                      deleted_name << "__deleted " << n->id;
//                                      n->setName(deleted_name.str());
//                                      return true;
//                                    } else {
//                                      return false;
//                                    } 
//                                 });
//     nodes.erase(end_it, nodes.end());
// }

boost::shared_ptr<NodeModel> PipelineModel::getNode(const std::string &name){
    auto it = std::find_if(nodes.begin(), nodes.end(),
                            [&name](boost::shared_ptr<NodeModel> &n) {
                                return n->getName() == name;
                                });
    if (it != nodes.end()){
        return *it;
    } else {
        return nullptr;
    }
}

// boost::shared_ptr<NodeModel> PipelineModel::getNode(NodeId id){
//     auto it = std::find_if(nodes.begin(), nodes.end(),
//                             [&id](boost::shared_ptr<NodeModel> &n) {
//                                 return n->id == id;
//                                 });
//     if (it != nodes.end()){
//         return *it;
//     } else {
//         return nullptr;
//     }
// }

/*
 * -------------XML Conversion
 */

typedef XmlRpc::XmlRpcValue RpcValue;

RpcValue PipelineModel::toXmlRpcValue() {
    RpcValue pipeline;
    RpcValue nodes_v;
    nodes_v.setSize(0); //marks the value as an array
    int i = 0;
    for (auto &node : nodes) {
        RpcValue node_v;
        node_v["name"] = node->getName();
        node_v["type"] = node->type.name;
        {
            RpcValue outputs_v;
            outputs_v.setSize(0);
            int j = 0;
            for (auto &output_name: node->getOutputNames()) {
                RpcValue output_v;
                auto &output = node->getOutput(output_name);
                output_v["name"] = output_name;
                RpcValue links_v;
                links_v.setSize(0);
                int k = 0;
                for (auto &input_r: output.outputs) {
                    auto &input = input_r.get();
                    //assert(input.node);
                    RpcValue link_v;
                    link_v["node_name"] = input.node->getName();
                    link_v["name"] = input.name;
                    links_v[k++] = link_v;
                }
                output_v["outputs"] = links_v;
                outputs_v[j++] = output_v;
            }
            node_v["outputs"] = outputs_v;
        }
        {
            int j = 0;
            RpcValue inputs_v;
            inputs_v.setSize(0);
            for (auto &input_name: node->getInputNames()) {
                RpcValue input_v;
                auto &input = node->getInput(input_name);
                if (input.input) {
                    input_v["name"] = input.name;
                    //nothing else required
                } else {
                    input_v["name"] = input.name;
                    input_v["value"] = input.value->toXmlRpcValue();
                    //technically redundant
                    input_v["type"] = input.value->getType();
                }
                inputs_v[j++] = input_v;
            }
            node_v["inputs"] = inputs_v;
        }

        nodes_v[i++] = node_v;
    }
    pipeline["nodes"] = nodes_v;
    return pipeline;
}

void PipelineModel::updateFromXmlRpcValue(RpcValue &value) {
    RpcValue nodes_v = value["nodes"];
    std::map<const std::string, boost::shared_ptr<NodeModel>> existing_nodes_by_name;
    std::map<const std::string, boost::shared_ptr<NodeModel>> nodes_by_name;
    std::map< std::pair<const std::string, const std::string>, std::pair<const std::string, const std::string> > edges;
    for (auto &node: nodes) {
        existing_nodes_by_name.insert(std::make_pair(node->getName(), node));
    }
    for (int i = 0; i < nodes_v.size(); i++) {
        auto &node_v = nodes_v[i];
        boost::shared_ptr<NodeModel> node;
        //create/get node as appropriate
        try {
            node = existing_nodes_by_name.at(node_v["name"]);
            if (node->type.name != std::string(node_v["type"])) {
                delNode(node_v["name"]);
                node = addNode(node_v["type"]);
                node->setName(node_v["name"]);
            }
        } catch (std::out_of_range) {
            node = addNode(node_v["type"]);
            node->setName(node_v["name"]);
        }
        //add to list of 'known' nodes
        nodes_by_name.insert(std::make_pair(node->getName(), node));
        //add inputs to  'known'  nodes
        RpcValue inputs_v = node_v["inputs"];
        for (int j = 0; j < inputs_v.size(); j++) {
            RpcValue input_v = inputs_v[j];
            if (input_v.hasMember("value")) {
                node->disconnectInput(input_v["name"]);
                node->setInputValue(input_v["name"], ParamTypeRegistry::getParamValue(input_v["type"], input_v["value"]));
            }
        }
        //add outputs to 'known' nodes
        RpcValue outputs_v = node_v["outputs"];
        for (int j = 0; j < inputs_v.size(); j++) {
            RpcValue output_v = outputs_v[j];
            RpcValue links_v = output_v["outputs"];
            for (int k = 0; k < links_v.size(); k++) {
                RpcValue link_v = links_v[k];
                edges.insert(std::make_pair(std::pair<const std::string, const std::string>(node_v["name"], output_v["name"]), 
                                            std::pair<const std::string, const std::string>(link_v["node_name"], link_v["name"])));
            }
        }
    }
    auto end = std::remove_if(nodes.begin(), nodes.end(), 
            [&nodes_by_name](boost::shared_ptr<NodeModel> &n) { return nodes_by_name.count(n->getName()) == 0; });
    nodes.erase(end, nodes.end());
}
