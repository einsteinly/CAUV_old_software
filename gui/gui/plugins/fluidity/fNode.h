/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <common/pipeline_model/node_model.h>

#include <liquid/node.h>

#include "managedElement.h"
#include "types.h"

namespace cauv{
namespace gui{
namespace f{

class FNodeInput;
class FNodeParamInput;
class FNodeOutput;
class ImageSource;

class FNode: public liquid::LiquidNode,
             public ManagedElement,
             public pipeline_model::NodeModel {
        Q_OBJECT
    public:
//         // - public typedefs
//         typedef std::vector<cauv::NodeInput> msg_node_in_list_t;
//         typedef std::map<cauv::LocalNodeOutput, msg_node_in_list_t> msg_node_output_map_t;
//         typedef std::map<cauv::LocalNodeInput, cauv::NodeOutput> msg_node_input_map_t;
//         typedef std::map<cauv::LocalNodeInput, cauv::ParamValue> msg_node_param_map_t;

    protected:
//         // - protected typedefs
//         typedef std::map<std::string, FNodeInput*> str_in_map_t;
//         typedef std::map<std::string, FNodeParamInput*> str_inparam_map_t;
//         typedef std::map<std::string, FNodeOutput*> str_out_map_t;
        FNode(const std::string type, Manager &m);
        void initIO();

    public:
        static boost::shared_ptr<FNode> makeFNode(const std::string type, Manager &m);

//         node_id_t id() const{ return m_node_id; }
//         NodeType::e nodeType() const{ return m_type; }

//         void setType(NodeType::e const&);
//         void setInputs(msg_node_input_map_t const&);
//         void setInputLinks(msg_node_input_map_t const&);
//         void setOutputs(msg_node_output_map_t const&);
//         void setOutputLinks(msg_node_output_map_t const&);
//         void setParams(msg_node_param_map_t const&);
//         void setParamLinks(msg_node_input_map_t const& inputs);
//         void connectOutputTo(const std::string& output, fnode_ptr, const std::string& input);
//         void disconnectOutputFrom(const std::string& output, fnode_ptr, const std::string& input);
//         void addImageDisplayOnInput(const std::string& input, boost::shared_ptr<ImageSource>);

        virtual void status(Status const& s, const std::string& status_information="");
        virtual void status(Status const& s, float const& throughput, float const& frequency, float const& time_taken, float const& time_ratio);
    
    Q_SIGNALS:
        void closed(pipeline_model::NodeModel&);
    
    public:
    // overridden virtual slots (don't need to be marked as slots):
        virtual void close();

    public Q_SLOTS:
        virtual void fadeAndRemove();
        virtual void remove();
        
        virtual void reExec();
        virtual void duplicate();
        virtual void toggleCollapsed();

    protected:
//         FNodeOutput* output(const std::string& id);
//         FNodeInput* input(const std::string& id);

//         void initFromMessage(boost::shared_ptr<NodeAddedMessage const> m);
        void initButtons();

    protected:
//         str_in_map_t       m_inputs;
//         str_inparam_map_t  m_params;
//         str_out_map_t      m_outputs;
        
        bool m_collapsed;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
