#ifndef __NODE_RENDERABLE_H__
#define __NODE_RENDERABLE_H__

#include <vector>
#include <set>

#include <common/debug.h>

#include "../container.h"
#include "draggable.h"

class NodeAddedMessage;
class NodeParametersMessage;
class Text;
class PipelineWidget;

class Node: public Draggable,
            public Container{
    typedef std::set<boost::shared_ptr<Renderable> > renderable_set_t;
    typedef renderable_set_t pv_set_t;
    typedef std::map<std::string, renderable_ptr_t> str_renderable_map_t;
    public:
        typedef PipelineWidget* pw_ptr_t;
        Node(container_ptr_t c, pw_ptr_t pw,
             boost::shared_ptr<NodeAddedMessage const> m);
        
        void setParams(boost::shared_ptr<NodeParametersMessage const> m);

        virtual void draw(bool);
        virtual bool mousePressEvent(MouseEvent const&);
        virtual void mouseReleaseEvent(MouseEvent const&);
        virtual void mouseMoveEvent(MouseEvent const&);
        virtual void mouseGoneEvent();
        virtual bool tracksMouse();
        virtual BBox bbox();
        
        // Node Stuff:
        int id() const;
        renderable_ptr_t outSocket(std::string const& output_id);
        renderable_ptr_t inSocket(std::string const& input_id);
        
        // implement Container:
        virtual Point referUp(Point const& p) const;
        virtual void postRedraw();
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position);
        virtual void removeMenu(menu_ptr_t);
        
        // specialized for known param types in node.cpp
        template<typename value_T>
        void paramValueChanged(std::string const& param, value_T const& v){
            error() << "unimplemented param type for" << param << v;
        }

    private:
        virtual void refreshLayout();

        pw_ptr_t m_pw;

        BBox m_bbox;
        BBox m_back;
        
        int m_node_id;
        std::string m_node_type;
    
        boost::shared_ptr<Text> m_title;
        pv_set_t m_params;
        str_renderable_map_t m_inputs;
        str_renderable_map_t m_outputs;

        bool m_suppress_dragable;

        renderable_set_t m_hovered;
};

#endif // ndef __NODE_RENDERABLE_H__

