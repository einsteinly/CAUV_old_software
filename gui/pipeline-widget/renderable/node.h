#ifndef __NODE_RENDERABLE_H__
#define __NODE_RENDERABLE_H__

#include <vector>
#include <set>

#include <generated/messages_fwd.h>
#include <debug/cauv_debug.h>

#include "../container.h"
#include "../pwTypes.h"
#include "draggable.h"

// eww, can't forward declare enums (or boost variants of standard library
// types...), have to drag in definitions
#include <generated/messages.h>

namespace cauv{

class NodeAddedMessage;
class NodeParametersMessage;
struct NodeOutput;
struct NodeInput;

namespace pw{

class Node: public Draggable,
            public Container{
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
        Node(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType::e const& nt);
        virtual ~Node(){ }

        void setType(NodeType::e const&);
        void setInputs(std::map<std::string, NodeOutput> const&);
        void setInputLinks(std::map<std::string, NodeOutput> const&);
        void setOutputs(std::map<std::string, std::vector<NodeInput> > const&);
        void setOutputLinks(std::map<std::string, std::vector<NodeInput> > const&);
        void setParams(std::map<std::string, NodeParamValue> const&);
        void setParams(boost::shared_ptr<NodeParametersMessage const> m);
        void setParamLinks(std::map<std::string, NodeOutput> const& inputs);        

        virtual void draw(drawtype_e::e flags);
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
        NodeType::e type() const;
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

        // specialized for known param types in node.cpp
        template<typename value_T>
        void paramValueChanged(std::string const& param, value_T const& v){
            error() << "unimplemented param type for" << param << "=" << v
                    <<", (n="<< id() <<")";
        }
    
    protected:
        renderable_set_t m_extra_stuff;

    private:
        pw_ptr_t m_pw;

        BBox m_bbox;
        BBox m_back;

        int m_node_id;
        NodeType::e m_node_type;

        text_ptr_t m_title;
        renderable_ptr_t m_closebutton;
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
} // namespace cauv

#endif // ndef __NODE_RENDERABLE_H__

