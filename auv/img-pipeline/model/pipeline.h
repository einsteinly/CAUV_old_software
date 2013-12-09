#pragma once

#include <string>
#include <array>
#include <XmlRpcValue.h>

#include "node_model.h"
#include "edge_model.h"

namespace cauv {

class PipelineModel {
    public:
    virtual boost::shared_ptr<NodeModel> addNode(const std::string type);
    virtual void delNode(const std::string &name);

    virtual XmlRpc::XmlRpcValue toXmlRpcValue();
    virtual void updateFromXmlRpcValue(XmlRpc::XmlRpcValue &value);

    static boost::shared_ptr<PipelineModel> fromXmlRpcValue(XmlRpc::XmlRpcValue &value);

    private:
    std::string name;
    std::set<std::string> node_names;
    std::vector<boost::shared_ptr<NodeModel>> nodes;
    friend class NodeModel;
};

}
