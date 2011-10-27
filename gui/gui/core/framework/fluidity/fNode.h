#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <map>
#include <vector>

#include <liquid/node.h>

#include "fluidity/managedElement.h"
#include "fluidity/types.h"

#include <generated/types/LocalNodeInput.h>
#include <generated/types/LocalNodeOutput.h>
#include <generated/types/NodeInput.h>
#include <generated/types/NodeOutput.h>
#include <generated/types/NodeParamValue.h>

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

    protected:
        node_id_t m_node_id;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
