#ifndef __NODE_IO_H__
#define __NODE_IO_H__

#include <boost/enable_shared_from_this.hpp>

#include "../renderable.h"
#include "../pwTypes.h"
#include "menu.h"

namespace cauv{
namespace pw{

class NodeIOBlob: public Renderable,
                  public boost::enable_shared_from_this<NodeIOBlob>{
    public:
        //typedef Node* node_ptr_t;
        NodeIOBlob(node_ptr_t node, pw_ptr_t pw, std::string const& name,
                   bool suppress_text = false);
        virtual ~NodeIOBlob(){ }

        virtual void draw(drawtype_e::e flags);
        virtual void mouseMoveEvent(MouseEvent const& m);
        virtual bool mousePressEvent(MouseEvent const& e);
        virtual void mouseReleaseEvent(MouseEvent const&);
        virtual void mouseGoneEvent();
        virtual bool tracksMouse();
        virtual BBox bbox();
        
        void status(int);
        node_id nodeId() const;

    protected:
        bool contains(Point const& x) const;

        node_wkptr_t m_node;
        pw_ptr_t m_pw;
        bool m_suppress_text;
        boost::shared_ptr<Text> m_text;

        double m_radius;
        double m_radius_squared;
        Colour m_normal_colour;
        Colour m_colour;

        bool m_mouseover;
};

class NodeInputBlob: public NodeIOBlob{
    public:
        NodeInputBlob(node_ptr_t d, pw_ptr_t p, std::string const& n,
                      bool suppress_text = false);
        virtual ~NodeInputBlob(){ }
        virtual bool mousePressEvent(MouseEvent const& e);
        std::string input() const;
};

class NodeInputParamBlob: public NodeInputBlob{
    public:
        NodeInputParamBlob(node_ptr_t d, pw_ptr_t p, std::string const& n);
        virtual ~NodeInputParamBlob(){ }
        std::string param() const;
};

class NodeOutputBlob: public NodeIOBlob{
    public:
        NodeOutputBlob(node_ptr_t d, pw_ptr_t p, std::string const& n);
        virtual ~NodeOutputBlob(){ }        
        std::string output() const;
};

class FloatingArcHandle: public Menu{
    public:
        FloatingArcHandle(pw_ptr_t pw, arc_ptr_t arc);
        virtual ~FloatingArcHandle(){ }
        
        virtual void draw(drawtype_e::e flags);
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
} // namespace cauv

#endif // ndef __NODE_IO_H__
