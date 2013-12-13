#pragma once

#include <map>
#include <set>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "edge_model.h"

namespace cauv {
namespace pipeline_model {

typedef int NodeId;

//Useful exceptions
class NoSuchParamException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class IncompatibleTypesException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

class DuplicateNodeNameException : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

//Describes a node type (i.e. its name, inputs and outputs)
class NodeModelType {
    public:
    NodeModelType();
    explicit NodeModelType(const std::string type);
    static std::map<std::string, const NodeModelType> node_types;


    virtual void addOutput(const std::string output_name, 
                           const std::string description,
                           ParamValue &default_value);
    virtual void addInput(const std::string input_name, 
                          const std::string description,
                          ParamValue &default_value);

    static void addType(const NodeModelType& type);

    const std::string name;
    std::map <const std::string, ParamModel> outputs;
    std::map <const std::string, ParamModel> inputs;
};

class PipelineModel;

//Describes an instance of a node, and how it is connected
class NodeModel : public boost::enable_shared_from_this<NodeModel> {
public:
    NodeModel(const std::string type, PipelineModel &pipeline);
    NodeModel(NodeModelType& type_, PipelineModel &pipeline_);

    virtual void connectOutput(const std::string output, InputModel&);
    virtual void connectInput(const std::string input, OutputModel&);

    virtual void disconnectOutput(const std::string output);
    virtual void disconnectInput(const std::string input);

    virtual InputModel& getInput(const std::string input_name) {
        return getEdge<InputModel>(inputs, type.inputs, input_name);
    }

    virtual OutputModel& getOutput(const std::string output_name) {
        return getEdge<OutputModel>(outputs, type.outputs, output_name);
    }

    NodeModel(const NodeModel &other) = delete;

    const NodeModelType &type;
    const NodeId id;

    std::string getName() { return name; };

    void setName(const std::string &new_name);

    const std::set<std::string> getOutputNames() const {
        return getEdgeNames<OutputModel>(outputs, type.outputs);
    }

    const std::set<std::string> getInputNames() const {
        return getEdgeNames<InputModel>(inputs, type.inputs);
    }

    //disconnect the node from other nodes
    virtual void isolate();

    virtual ~NodeModel();
protected:
private:
    template< typename EdgeModelType >
    const std::set<std::string> getEdgeNames( const std::map <const std::string, EdgeModelType> &instance_map,
                                        const std::map <const std::string, ParamModel> &type_map ) const {
        std::set<std::string> names;
        for (auto &edge: instance_map) {
            names.insert(edge.first);
        }
        for (auto &edge: type_map) {
            names.insert(edge.first);
        }
        return names;
    }

    template < typename EdgeModelType >
    EdgeModelType& getEdge(std::map <const std::string, EdgeModelType> &instance_map,
                                 const std::map <const std::string, ParamModel> &type_map,
                                 const std::string edge) {
        try {
            return instance_map.at(edge);
        } catch (std::out_of_range) {
            //lazy binding because shared_from_this does not work in the
            //constructor
            //blah blah blah exceptions control flow etc, this is clearer and
            //faster in the common case
            try {
                instance_map.insert(std::make_pair(edge, EdgeModelType(type_map.at(edge), shared_from_this())));
                return instance_map.at(edge);
            } catch (std::out_of_range) {
                throw NoSuchParamException("No such edge '" + edge + "' To node '" + name + "'");
            }
        }
    }

    std::string name;
    std::map <const std::string, OutputModel> outputs;
    std::map <const std::string, InputModel> inputs;
    static int current_id;
    PipelineModel &pipeline;
};

}
}
