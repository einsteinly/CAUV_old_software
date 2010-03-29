#ifndef __NODE_RENDERABLE_H__
#define __NODE_RENDERABLE_H__

#include <vector>

#include <common/debug.h>

#include "../container.h"
#include "draggable.h"

class NodeAddedMessage;
class NodeParametersMessage;
class Text;
class PipelineWidget;

class Node: public Draggable,
            public Container{
    typedef std::list<boost::shared_ptr<Renderable> > pv_list_t;
    public:
        typedef PipelineWidget* pw_ptr_t;
        Node(container_ptr_t c, pw_ptr_t pw,
             boost::shared_ptr<NodeAddedMessage const> m);
        
        void setParams(boost::shared_ptr<NodeParametersMessage const> m);

        virtual void draw(bool);
        virtual void mousePressEvent(MouseEvent const&);
        virtual bool tracksMouse();
        virtual BBox bbox();

        int id() const;
        
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
        pw_ptr_t m_pw;

        BBox m_bbox;
        
        int m_node_id;
        std::string m_node_type;
    
        boost::shared_ptr<Text> m_title;
        pv_list_t m_params;
};

#endif // ndef __NODE_RENDERABLE_H__

