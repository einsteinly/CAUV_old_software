/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <liquid/node.h>

#include "fluidity/managedElement.h"
#include "fluidity/types.h"


#include <generated/types/LocalNodeInput.h>
#include <generated/types/LocalNodeOutput.h>
#include <generated/types/NodeInput.h>
#include <generated/types/NodeOutput.h>
#include <generated/types/NodeParamValue.h>
#include <generated/types/NodeAddedMessage.h>
#include <generated/types/NodeType.h>

namespace cauv{
namespace gui{
namespace f{

class FNode: public liquid::LiquidNode,
             public ManagedElement{
    public:
        // - public typedefs
        typedef std::vector<cauv::NodeInput> msg_node_in_list_t;
        typedef std::map<cauv::LocalNodeOutput, msg_node_in_list_t> msg_node_output_map_t;
        typedef std::map<cauv::LocalNodeInput, cauv::NodeOutput> msg_node_input_map_t;
        typedef std::map<cauv::LocalNodeInput, cauv::NodeParamValue> msg_node_param_map_t;

    private:
        // - private typedefs
        

    public:
        FNode(Manager& m, node_id_t id);
        FNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p);

        void setType(NodeType::e const&);
        void setInputs(msg_node_input_map_t const&);
        void setInputLinks(msg_node_input_map_t const&);
        void setOutputs(msg_node_output_map_t const&);
        void setOutputLinks(msg_node_output_map_t const&);
        void setParams(msg_node_param_map_t const&);
        void setParamLinks(msg_node_input_map_t const& inputs);
    
    public Q_SLOTS:

    protected:
        void initFromMessage(boost::shared_ptr<NodeAddedMessage const> m);
    

    protected:
        node_id_t m_node_id;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
