/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <liquid/node.h>

#include "managedElement.h"
#include "types.h"

#include <generated/types/LocalNodeInput.h>
#include <generated/types/LocalNodeOutput.h>
#include <generated/types/NodeInput.h>
#include <generated/types/NodeOutput.h>
#include <generated/types/ParamValue.h>
#include <generated/types/NodeAddedMessage.h>
#include <generated/types/NodeType.h>
#include <generated/types/NodeOutputArc.h>
#include <generated/types/NodeInputArc.h>

namespace cauv{
namespace gui{
namespace f{

class FNodeInput;
class FNodeParamInput;
class FNodeOutput;
class ImageSource;

class FNode: public liquid::LiquidNode,
             public ManagedElement{
        Q_OBJECT
    public:
        // - public typedefs
        typedef std::vector<cauv::NodeInput> msg_node_in_list_t;
        typedef std::map<cauv::LocalNodeOutput, msg_node_in_list_t> msg_node_output_map_t;
        typedef std::map<cauv::LocalNodeInput, cauv::NodeOutput> msg_node_input_map_t;
        typedef std::map<cauv::LocalNodeInput, cauv::ParamValue> msg_node_param_map_t;

    protected:
        // - protected typedefs
        typedef std::map<std::string, FNodeInput*> str_in_map_t;
        typedef std::map<std::string, FNodeParamInput*> str_inparam_map_t;
        typedef std::map<std::string, FNodeOutput*> str_out_map_t;

    public:
        FNode(Manager& m, node_id_t id, NodeType::e const& type);
        FNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p);

        node_id_t id() const{ return m_node_id; }
        NodeType::e nodeType() const{ return m_type; }

        void setType(NodeType::e const&);
        void setInputs(msg_node_input_map_t const&);
        void setInputLinks(msg_node_input_map_t const&);
        void setOutputs(msg_node_output_map_t const&);
        void setOutputLinks(msg_node_output_map_t const&);
        void setParams(msg_node_param_map_t const&);
        void setParamLinks(msg_node_input_map_t const& inputs);
        void connectOutputTo(std::string const& output, fnode_ptr, std::string const& input);
        void disconnectOutputFrom(std::string const& output, fnode_ptr, std::string const& input);
        void addImageDisplayOnInput(std::string const& input, boost::shared_ptr<ImageSource>);

        virtual void status(Status const& s, std::string const& status_information="");
        virtual void status(Status const& s, float const& throughput, float const& frequency, float const& time_taken, float const& time_ratio);
    
    Q_SIGNALS:
        void closed(node_id_t const);
    
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
        FNodeOutput* output(std::string const& id);
        FNodeInput* input(std::string const& id);

        void initFromMessage(boost::shared_ptr<NodeAddedMessage const> m);
        void initButtons();

    protected:
        node_id_t m_node_id;
        NodeType::e m_type;
    
        str_in_map_t       m_inputs;
        str_inparam_map_t  m_params;
        str_out_map_t      m_outputs;
        
        // NB: do not assume these always correspond to m_inputs and m_outputs,
        // this is not the case while arcs are pending addition or removal.
        std::vector<cauv::NodeInputArc> m_input_links; // includes parameters
        std::vector<cauv::NodeOutputArc> m_output_links;

        bool m_collapsed;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
