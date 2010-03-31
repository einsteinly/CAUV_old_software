#ifndef __NODE_IO_H__
#define __NODE_IO_H__

#include <boost/enable_shared_from_this.hpp>

#include "../renderable.h"
#include "../pwTypes.h"
#include "menu.h"

namespace pw{

class NodeIOBlob: public Renderable,
                  public boost::enable_shared_from_this<NodeIOBlob>{
    public:
        NodeIOBlob(Node* node, pw_ptr_t pw, std::string const& name);

        virtual void draw(bool picking);
        virtual void mouseMoveEvent(MouseEvent const& m);
        virtual bool mousePressEvent(MouseEvent const& e);
        virtual void mouseReleaseEvent(MouseEvent const&);
        virtual void mouseGoneEvent();
        virtual bool tracksMouse();
        virtual BBox bbox();

        node_id nodeId() const;

    protected:
        bool contains(Point const& x) const;

        node_ptr_t m_node;
        pw_ptr_t m_pw;
        boost::shared_ptr<Text> m_text;

        double m_radius;
        double m_radius_squared;
        Colour m_colour;
        Colour m_colour_hl;
        Colour m_outline_colour;

        bool m_mouseover;
};

class NodeInputBlob: public NodeIOBlob{
    public:
        NodeInputBlob(Node* d, pw_ptr_t p, std::string const& n);
        std::string input() const;
};

class NodeOutputBlob: public NodeIOBlob{
    public:
        NodeOutputBlob(Node* d, pw_ptr_t p, std::string const& n);
        std::string output() const;
};

class FloatingArcHandle: public Menu{
    public:
        FloatingArcHandle(pw_ptr_t pw, arc_ptr_t arc);
        
        virtual void draw(bool);
        virtual void mouseReleaseEvent(MouseEvent const& e);
        virtual void mouseMoveEvent(MouseEvent const& e);
        virtual void mouseGoneEvent();
        virtual BBox bbox();

    private:
        pw_ptr_t m_pw;
        arc_ptr_t m_arc;
        Point m_click_pos;
};

} // namespace pw

#endif // ndef __NODE_IO_H__
