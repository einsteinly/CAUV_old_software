#ifndef __NODE_RENDERABLE_H__
#define __NODE_RENDERABLE_H__

#include <vector>

#include "../container.h"
#include "draggable.h"

class NodeAddedMessage;
class NodeParametersMessage;
class Text;

class Node: public Draggable,
            public Container{
    typedef std::list<boost::shared_ptr<Renderable> > pv_list_t;
    public:
        Node(container_ptr_t c, boost::shared_ptr<NodeAddedMessage const> m);
        
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

    private:
        BBox m_bbox;
        
        int m_node_id;
        std::string m_node_type;
    
        boost::shared_ptr<Text> m_title;
        pv_list_t m_params;
};

#endif // ndef __NODE_RENDERABLE_H__

