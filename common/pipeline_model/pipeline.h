#pragma once

#include <boost/shared_ptr.hpp>

#include <string>
#include <array>
#include <XmlRpcValue.h>

#include "node_model.h"
#include "edge_model.h"

namespace cauv {
    
namespace pipeline_model {
    
class PipelineModel {
    //(basic) pipeline model, providing methods to convert to/from xml
public:
    PipelineModel(const std::string& pipeline_name);
    
    //retrieve information
    const std::string& pipelineName();
    
    //manipulate nodes
    virtual boost::shared_ptr<NodeModel> addNode(const std::string type);
    virtual void delNode(const std::string &name);
    virtual void delNodeById(NodeId id);
    virtual boost::shared_ptr<NodeModel> getNode(const std::string &name);
    virtual boost::shared_ptr<NodeModel> getNode(NodeId id);
    
    //xml conversion
    virtual XmlRpc::XmlRpcValue toXmlRpcValue();
    virtual void updateFromXmlRpcValue(XmlRpc::XmlRpcValue &value);

    static boost::shared_ptr<PipelineModel> fromXmlRpcValue(XmlRpc::XmlRpcValue &value);

private:
    std::string pipeline_name;
    std::set<std::string> node_names; //list of node_names, used to ensure no two nodes have same name
    std::vector<boost::shared_ptr<NodeModel>> nodes;
    friend class NodeModel;
};

}//pipeline_model
}//cauv
