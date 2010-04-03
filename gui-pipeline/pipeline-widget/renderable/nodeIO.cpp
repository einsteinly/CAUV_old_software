#include "nodeIO.h"

#include <boost/make_shared.hpp>

#include "../pipelineWidget.h"
#include "text.h"
#include "arc.h"
#include "node.h"

using namespace pw;

const static Colour Normal_Colour(0.2, 0.4, 0.6, 0.5);
const static Colour Outline_Colour_Hint(0, 0.5);
const static Colour Mouseover_Colour_Hint(1, 0.5);
const static Colour New_Hint(0, 1, 0, 1);
const static Colour Demanded_Hint(1, 1, 0, 1);
const static Colour Invalid_Hint(0, 0, 0, 0.5);

NodeIOBlob::NodeIOBlob(node_ptr_t node, pw_ptr_t pw, std::string const& name)
    : Renderable(node), m_node(node), m_pw(pw),
      m_text(boost::make_shared<Text>(node, name)),
      m_radius(6), m_radius_squared(m_radius*m_radius),
      m_colour(Normal_Colour),
      m_mouseover(false){
    // put middle at 0 rather than baseline
    m_text->m_pos.y = -(m_text->bbox().min.y + m_text->bbox().h()/2);
}

void NodeIOBlob::draw(bool picking){
    if(m_mouseover)
        glColor(m_colour & Mouseover_Colour_Hint);
    else
        glColor(m_colour);
    glCircle(m_radius);

    glColor(m_colour & Outline_Colour_Hint);
    glLineWidth(1);
    glCircleOutline(m_radius);

    glTranslatef(m_text->m_pos);
    m_text->draw(picking);
}

void NodeIOBlob::mouseMoveEvent(MouseEvent const& m){
    if(!m_mouseover && contains(m.pos)){
        m_mouseover = true;
        m_context->postRedraw();
    }else if(m_mouseover && !contains(m.pos)){
        m_mouseover = false;
        m_context->postRedraw();
    }
}

bool NodeIOBlob::mousePressEvent(MouseEvent const& e){
    if(e.pos.sxx() < m_radius_squared){
        // circley bit hit
        debug() << BashColour::Green << "creating new arc handle n="
                << m_node->id() << "io=" << *m_text<< "this=" << this;
        arc_ptr_t arc = m_node->newArc(shared_from_this(), renderable_ptr_t());
        menu_ptr_t handle = boost::make_shared<FloatingArcHandle>(m_pw, arc);
        arc->m_dst = handle;
        // (true = start out 'pressed', ie being dragged)
        m_context->postMenu(handle, m_context->referUp(m_pos), true);
        return true;
    }else{
        return false;
    }
}

void NodeIOBlob::mouseReleaseEvent(MouseEvent const&){
}

void NodeIOBlob::mouseGoneEvent(){
    if(m_mouseover){
        m_mouseover = false;
        m_context->postRedraw();
    }
}

bool NodeIOBlob::tracksMouse(){
    return true;
}

BBox NodeIOBlob::bbox(){
    return BBox(-m_radius, -m_radius, m_radius, m_radius) |
           (m_text->bbox() + m_text->m_pos);
}

bool NodeIOBlob::contains(Point const& x) const{
    if(x.sxx() < m_radius_squared)
        return true;
    return false;
}

void NodeIOBlob::status(int s){
    m_colour = Normal_Colour;
    if(s & NodeIOStatus::New) m_colour &= New_Hint;
    if(!(s & NodeIOStatus::Valid)) m_colour &= Invalid_Hint;
    if(s & NodeIOStatus::Demanded) m_colour &= Demanded_Hint;
    m_context->postRedraw(); 
}

node_id NodeIOBlob::nodeId() const{
    return m_node->id();
}


NodeInputBlob::NodeInputBlob(node_ptr_t d, pw_ptr_t p, std::string const& n)
    : NodeIOBlob(d, p, n){
    m_text->m_pos.x = -m_text->bbox().min.x + m_radius + 3;
}

std::string NodeInputBlob::input() const{
    return *m_text;
}

NodeOutputBlob::NodeOutputBlob(node_ptr_t d, pw_ptr_t p, std::string const& n)
    : NodeIOBlob(d, p, n){
    m_text->m_pos.x = -m_text->bbox().max.x - m_radius - 3;
}

std::string NodeOutputBlob::output() const{
    return *m_text;
}


FloatingArcHandle::FloatingArcHandle(pw_ptr_t pw, arc_ptr_t arc)
    : Menu(pw), m_pw(pw), m_arc(arc), m_click_pos(){
    m_click_pos = Point();
}

void FloatingArcHandle::draw(bool){
    glColor(Colour(0.6, 0.7, 0.8, 0.5));
    glTranslatef(0, 0, 0.3);
    glCircle(5.0);
    glCircleOutline(3.0);
}

void FloatingArcHandle::mouseReleaseEvent(MouseEvent const&){
    // remove first, so that this isn't returned by the pick
    m_context->removeMenu(shared_from_this());
    //remove the arc: arc is re-added when we receive an ArcAddedMessage        
    m_context->remove(m_arc);

    renderable_ptr_t hit = m_pw->pick(m_pos);
    debug() << BashColour::Green
            << "FloatingArcHandle released on" << hit;

    boost::shared_ptr<NodeIOBlob> dropped_on_io;
    if(dropped_on_io = boost::dynamic_pointer_cast<NodeIOBlob>(hit)){
        debug() << "dropped on IO" << dropped_on_io;
        m_arc->m_dst = dropped_on_io;
        m_pw->sendMessage(
            boost::make_shared<AddArcMessage>(m_arc->from(), m_arc->to())
        );
    }else{
        debug() << "not an IO";
    }
    m_arc.reset();
}

void FloatingArcHandle::mouseMoveEvent(MouseEvent const& e){
    if(e.buttons & Qt::LeftButton){
        m_pos += e.pos - m_click_pos;
        // need a re-draw
        m_context->postRedraw();
    }
}

void FloatingArcHandle::mouseGoneEvent(){
    m_context->removeMenu(shared_from_this());
    debug() << BashColour::Green << "FloatingArcHandle lost mouse"; 
    m_context->remove(m_arc);
    m_arc.reset();
}

BBox FloatingArcHandle::bbox(){
    // big enough that the mouse can't move outside it during a drag
    // operation
    //return BBox(-100, -100, 100, 100);
    return BBox();
}


