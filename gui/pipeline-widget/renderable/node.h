#ifndef __NODE_RENDERABLE_H__
#define __NODE_RENDERABLE_H__

#include <vector>
#include <set>

#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <debug/cauv_debug.h>
#include <generated/types/NodeType.h>
#include <generated/types/ParamValue.h>
#include <generated/types/NodeOutput_fwd.h>
#include <generated/types/NodeInput_fwd.h>
#include <generated/types/LocalNodeInput_fwd.h>
#include <generated/types/LocalNodeOutput_fwd.h>
#include <generated/types/SetNodeParameterMessage.h>

#include "../container.h"
#include "../pwTypes.h"
#include "../pipelineWidget.h"
#include "draggable.h"

namespace cauv{

class NodeAddedMessage;
class NodeParametersMessage;

namespace gui{
namespace pw{

class Node: public Draggable,
            public Container,
            public boost::enable_shared_from_this<Node>{
    public:
        // public typedefs
        typedef std::vector<NodeInput> msg_node_in_list_t;
        typedef std::map<LocalNodeOutput, msg_node_in_list_t> msg_node_output_map_t;
        typedef std::map<LocalNodeInput, NodeOutput> msg_node_input_map_t;
        typedef std::map<LocalNodeInput, ParamValue> msg_node_param_map_t;
    
    private:
        // private typedefs used internally:
        typedef std::set<renderable_ptr_t> renderable_set_t;
        typedef std::map<std::string, renderable_ptr_t> str_renderable_map_t;
        typedef boost::shared_ptr<NodeInputBlob> in_ptr_t;
        typedef boost::shared_ptr<NodeInputParamBlob> inparam_ptr_t;
        typedef boost::shared_ptr<NodeOutputBlob> out_ptr_t;
        typedef std::map<std::string, in_ptr_t> str_in_map_t;
        struct InParamPVPair{
            inparam_ptr_t inblob;
            boost::shared_ptr<PVPairEditableBase> pvpair;
        };
        typedef std::map<std::string, InParamPVPair> str_inparam_map_t;
        typedef std::map<std::string, out_ptr_t> str_out_map_t;
        // iterator typedefs, sorry!
        typedef Node::str_renderable_map_t::const_iterator sr_map_iter_t;
        
        // friends
        template<typename T, typename C>
        friend std::basic_ostream<T,C>& operator<<(std::basic_ostream<T,C>&, Node const&);
        
    public:
        Node(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m);
        Node(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType const& nt);
        virtual ~Node(){ }

        void initFromMessage(boost::shared_ptr<NodeAddedMessage const> m);
        void setType(NodeType const&);
        void setInputs(msg_node_input_map_t const&);
        void setInputLinks(msg_node_input_map_t const&);
        void setOutputs(msg_node_output_map_t const&);
        void setOutputLinks(msg_node_output_map_t const&);
        void setParams(msg_node_param_map_t const&);
        void setParams(boost::shared_ptr<NodeParametersMessage const> m);
        void setParamLinks(msg_node_input_map_t const& inputs);

        virtual void draw(drawtype_e flags);
        virtual bool mousePressEvent(MouseEvent const&);
        virtual void mouseReleaseEvent(MouseEvent const&);
        virtual void mouseMoveEvent(MouseEvent const&);
        virtual void mouseGoneEvent();
        virtual bool tracksMouse();
        virtual BBox bbox();

        // Node Stuff:
        void close();
        void exec();
        int id() const;
        NodeType type() const;
        renderable_ptr_t outSocket(std::string const& output_id);
        renderable_ptr_t inSocket(std::string const& input_id);
        arc_ptr_t newArc(renderable_wkptr_t src, renderable_wkptr_t dst);
        void status(int s);
        void inputStatus(std::string const& input_id, int s);
        void outputStatus(std::string const& output_id, int s);

        // implement Container:
        virtual Point referUp(Point const& p) const;
        virtual void postRedraw(float delay);
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position,
                              bool pressed=false);
		virtual void postText(const std::string &text, const std::string &font);
        virtual void removeMenu(menu_ptr_t);
        virtual void remove(renderable_ptr_t);
        virtual void refreshLayout();

        template<typename value_T>
        void paramValueChanged(std::string const& param, value_T const& v){
            debug() << "Node::paramValueChanged" << param << v;
            boost::shared_ptr<SetNodeParameterMessage> sp =
                boost::make_shared<SetNodeParameterMessage>();
            ParamValue pv = v;
            sp->pipelineName(m_pw->pipelineName());
            sp->nodeId(m_node_id);
            sp->paramId(param);
            sp->value(pv);
            m_pw->send(sp);
        }
    
    protected:
        renderable_set_t m_extra_stuff;

    private:
        pw_ptr_t m_pw;

        BBox m_bbox;
        BBox m_back;

        int m_node_id;
        NodeType m_node_type;

        text_ptr_t m_title;
        renderable_ptr_t m_closebutton;
        renderable_ptr_t m_idtext;
        renderable_ptr_t m_execbutton;
        str_in_map_t m_inputs;
        str_inparam_map_t m_params;        
        str_out_map_t m_outputs;

        bool m_suppress_draggable;

        renderable_set_t m_hovered;
        renderable_set_t m_pressed;

        Colour m_bg_col;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __NODE_RENDERABLE_H__

