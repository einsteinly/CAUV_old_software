#include "node.h"

#include <sstream>

#include <boost/enable_shared_from_this.hpp>

#include <QtOpenGL>

#include <common/cauv_utils.h>
#include <common/messages.h>

#include "../pipelineWidget.h"
#include "text.h"
#include "editText.h"
#include "arc.h"
#include "nodeIO.h"

namespace pw{

// TODO: own header?
template<typename value_T>
class PVPair: public Renderable{
    public:
        typedef Node* node_ptr_t;
        PVPair(node_ptr_t n, std::string const& param, value_T const& value)
            : Renderable(n), m_node(n), m_bbox(),
              m_param(boost::make_shared<Text>(n, param)),
              m_equals(boost::make_shared<Text>(n, "=")),
              m_value(boost::make_shared<Text>(n, to_string(value))){
            updateBbox();
        }

        void draw(bool picking){
            glPushMatrix();
            glTranslatef(m_param->m_pos);
            m_param->draw(picking);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(m_equals->m_pos);
            m_equals->draw(picking);
            glPopMatrix();
            
            glPushMatrix();
            glTranslatef(m_value->m_pos);
            m_value->draw(picking);
            glPopMatrix();
        }

        virtual bool mousePressEvent(MouseEvent const& e){
            MouseEvent referred(e, m_value);
            if(m_value->bbox().contains(referred.pos)){
                debug() << "got value hit";
                BBox edit_box = m_value->bbox();
                edit_box.max.x += 10;
                m_context->postMenu(
                    boost::make_shared<EditText<PVPair const&> >(
                        m_context, *m_value, edit_box, &onValueChanged, boost::ref(*this)
                    ),
                    m_context->referUp(m_pos + m_value->m_pos)
                );
                m_context->postRedraw();
                return true;
            }
            return false;
        }

        virtual BBox bbox(){
            return m_bbox;
        }


    private:
        static void onValueChanged(PVPair const& pvp, std::string const& s){
            std::istringstream is(s);
            value_T v;
            is >> v;
            debug() << "PVPair: Edit done:" << s << "->" << v;
            pvp.m_node->paramValueChanged(*pvp.m_param, v);
        }

        void updateBbox(){
            m_bbox = m_param->bbox();
            m_equals->m_pos.x = m_bbox.max.x + 3 - m_equals->bbox().min.x;
            m_bbox |= m_equals->bbox() + m_equals->m_pos;
            m_value->m_pos.x = m_bbox.max.x + 3 - m_value->bbox().min.x;
            m_bbox |= m_value->bbox() + m_value->m_pos;
        }

        node_ptr_t m_node;

        BBox m_bbox;

        boost::shared_ptr<Text> m_param;
        boost::shared_ptr<Text> m_equals;
        boost::shared_ptr<Text> m_value;
};

} // namespace pw

using namespace pw;

Node::Node(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m)
    : Draggable(c), m_pw(pw), m_bbox(), m_node_id(m->nodeId()),
      m_node_type(to_string(m->nodeType())),
      m_title(boost::make_shared<Text>(c, m_node_type)),
      m_suppress_draggable(false){
    m_contents.push_back(m_title);

    std::map<std::string, std::vector<NodeInput> >::const_iterator i;
    for(i = m->outputs().begin(); i != m->outputs().end(); i++){
        debug() << BashColour::Brown << *i;
        renderable_ptr_t t = boost::make_shared<NodeOutputBlob>(this, m_pw, i->first);
        m_outputs[i->first] = t;
        m_contents.push_back(t);
        // add the arcs (if any) from this output:
        std::vector<NodeInput>::const_iterator k;
        for(k = i->second.begin(); k != i->second.end(); k++){
            m_pw->addArc(t, k->node, k->input);
        }
    }
    
    std::map<std::string, NodeOutput>::const_iterator j;
    for(j = m->inputs().begin(); j != m->inputs().end(); j++){
        debug() << BashColour::Brown << *j;
        renderable_ptr_t t = boost::make_shared<NodeInputBlob>(this, m_pw, j->first);
        m_inputs[j->first] = t;
        m_contents.push_back(t);
        // if connected, add the arc to this input:
        if(j->second.node)
            m_pw->addArc(j->second.node, j->second.output, t);
    }
    
    refreshLayout();
}

static boost::shared_ptr<Renderable> makePVPair(
    Node *n, std::pair<std::string, NodeParamValue> const& p){
    switch((ParamType::e) p.second.type){
        case ParamType::Int32:
            return boost::make_shared<PVPair<int> >(
                    n, p.first, p.second.intValue
                );
        case ParamType::Float:
            return boost::make_shared<PVPair<float> >(
                    n, p.first, p.second.floatValue
                );
        default:
            error() << "unknown ParamType";
        case ParamType::String:
            return boost::make_shared<PVPair<std::string> >(
                    n, p.first, p.second.stringValue
                );
    }
}

void Node::setParams(boost::shared_ptr<NodeParametersMessage const> m){
    if(m_node_id != m->nodeId()){
        warning() << "parameters not for this node";
        return;
    }
    // remove any old parameters:
    renderable_list_t::iterator k, j;
    for(k = m_contents.begin(); k != m_contents.end(); k=j){
        j = k;
        j++;
        if(m_params.count(*k))
            m_contents.erase(k);
    }
    m_params.clear();

    std::map<std::string, NodeParamValue>::const_iterator i;    
    for(i = m->values().begin(); i != m->values().end(); i++){
        boost::shared_ptr<Renderable> t = makePVPair(this, *i);
        m_params.insert(t);
        m_contents.push_back(t);
    }
    refreshLayout();
}

bool Node::mousePressEvent(MouseEvent const& e){
    renderable_list_t::const_iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        MouseEvent referred(e, *i);
        if((*i)->bbox().contains(referred.pos))
            if((*i)->mousePressEvent(referred)){
                m_suppress_draggable = true;
                return true;
            }
    }
    return Draggable::mousePressEvent(e);
}

void Node::mouseReleaseEvent(MouseEvent const& e){
    m_suppress_draggable = false;
    Draggable::mouseReleaseEvent(e);
}

void Node::mouseMoveEvent(MouseEvent const& e){
    renderable_list_t::const_iterator i;
    renderable_set_t now_hovered;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        MouseEvent referred(e, *i);
        if((*i)->tracksMouse() && (*i)->bbox().contains(referred.pos)){
            (*i)->mouseMoveEvent(referred);
            now_hovered.insert(*i);
        }
    }
    renderable_set_t::const_iterator j;
    for(j = m_hovered.begin(); j != m_hovered.end(); j++){
        if(!now_hovered.count(*j))
            (*j)->mouseGoneEvent();
    }
    m_hovered = now_hovered;
    if(!m_suppress_draggable)
        Draggable::mouseMoveEvent(e);
}

void Node::mouseGoneEvent(){
    renderable_set_t::const_iterator i;
    for(i = m_hovered.begin(); i != m_hovered.end(); i++)
        (*i)->mouseGoneEvent();
    m_hovered.clear();
    if(!m_suppress_draggable)
        Draggable::mouseGoneEvent();
}

void Node::draw(bool picking){
    if(m_mouseover)
        glColor4f(0.1, 0.5, 0.1, 0.8);
    else
        glColor4f(0.8, 0.8, 0.8, 0.8);
    glBox(m_back);
    
    Container::draw(picking);
}

bool Node::tracksMouse(){
    return true;
}

BBox Node::bbox(){
    return m_bbox;
}

int Node::id() const{
    return m_node_id;
}

renderable_ptr_t Node::outSocket(std::string const& id){
    str_renderable_map_t::const_iterator i = m_outputs.find(id);
    if(i != m_outputs.end()){
        return i->second;
    }else{
        warning() << "unknown output:" << id << "on node" << m_node_id;
        return renderable_ptr_t();
    }
}

renderable_ptr_t Node::inSocket(std::string const& id){
    str_renderable_map_t::const_iterator i = m_inputs.find(id);
    if(i != m_inputs.end()){
        return i->second;
    }else{
        warning() << "unknown input:" << id << "on node" << m_node_id;
        return renderable_ptr_t();
    }
}

arc_ptr_t Node::newArc(renderable_wkptr_t src, renderable_wkptr_t dst){
    return m_pw->addArc(src, dst);
}

Point Node::referUp(Point const& p) const{
    return m_context->referUp(p + m_pos);
}

void Node::postRedraw(){
    m_context->postRedraw();
}

void Node::postMenu(menu_ptr_t m, Point const& p, bool r){
    m_context->postMenu(m, p, r);
}

void Node::removeMenu(menu_ptr_t m){
    m_context->removeMenu(m);
}

void Node::remove(renderable_ptr_t r){
    error() << __func__ << "unimplemented";
}

namespace pw{

template<>
void Node::paramValueChanged<int>(std::string const& p, int const& v){
    debug() << "Node::paramValueChanged<int>" << p << v;
    boost::shared_ptr<SetNodeParameterMessage> sp =
        boost::make_shared<SetNodeParameterMessage>();
    NodeParamValue pv = {0};

    sp->nodeId(m_node_id);
    sp->paramId(p);
    pv.type = ParamType::Int32;
    pv.intValue = v;
    sp->value(pv);
    m_pw->sendMessage(sp);
}

template<>
void Node::paramValueChanged<float>(std::string const& p, float const& v){
    debug() << "Node::paramValueChanged<float>" << p << v;
    boost::shared_ptr<SetNodeParameterMessage> sp =
        boost::make_shared<SetNodeParameterMessage>();
    NodeParamValue pv = {0};

    sp->nodeId(m_node_id);
    sp->paramId(p);
    pv.type = ParamType::Float;
    pv.floatValue = v;
    sp->value(pv);
    m_pw->sendMessage(sp);
}

template<>
void Node::paramValueChanged<std::string>(std::string const& p, std::string const& v){
    debug() << "Node::paramValueChanged<string>" << p << v;
    boost::shared_ptr<SetNodeParameterMessage> sp =
        boost::make_shared<SetNodeParameterMessage>();
    NodeParamValue pv = {0};

    sp->nodeId(m_node_id);
    sp->paramId(p);
    pv.type = ParamType::String;
    pv.stringValue = v;
    sp->value(pv);
    m_pw->sendMessage(sp);
}

} // namespace pw


void Node::refreshLayout(){
    int border = 3;
    int lead = 6;
    int section_lead = 10;
    int io_overhang = 6;
    int prev_height = roundA(m_title->bbox().h());
    int y_pos = -(prev_height+section_lead);
    int param_indent = 6;

    m_title->m_pos.x = -m_title->bbox().min.x;
    m_title->m_pos.y = -m_title->bbox().max.y;
    m_back = m_title->bbox() + m_title->m_pos;
    m_back.max.y += border;
    m_back.min.x -= border;
    m_back.max.x += border;

    str_renderable_map_t::const_iterator i;
    for(i = m_inputs.begin(); i != m_inputs.end(); i++, y_pos -= (prev_height+lead)){
        renderable_ptr_t r = i->second;
        r->m_pos.y = y_pos - roundA(r->bbox().max.y);
        r->m_pos.x = m_back.min.x - io_overhang - r->bbox().min.x;
        prev_height = roundA(r->bbox().h());
        m_back |= r->bbox() + r->m_pos + Point(io_overhang, 0);
    }
    y_pos -= section_lead - lead;

    pv_set_t::const_iterator j;    
    for(j = m_params.begin(); j != m_params.end(); j++, y_pos -= (prev_height+lead)){
        (*j)->m_pos.y = y_pos - roundA((*j)->bbox().max.y);
        (*j)->m_pos.x = param_indent;
        prev_height = roundA((*j)->bbox().h());
        m_back |= (*j)->bbox() + (*j)->m_pos;
    }
    y_pos -= section_lead - lead;
    
    for(i = m_outputs.begin(); i != m_outputs.end(); i++, y_pos -= (prev_height+lead)){
        renderable_ptr_t r = i->second;
        r->m_pos.y = y_pos - roundA(r->bbox().max.y);
        if(r->bbox().w() - io_overhang > m_back.w())
            r->m_pos.x = m_back.min.x - r->bbox().min.x;
        else
            r->m_pos.x = m_back.max.x + io_overhang - r->bbox().max.x;
        m_back |= r->bbox() + r->m_pos - Point(io_overhang, 0);
        prev_height = roundA(r->bbox().h());
    }

    m_back.min.y -= lead;

    m_bbox = m_back;
    if(m_inputs.size())
        m_bbox.min.x -= param_indent;
    if(m_outputs.size())
        m_bbox.max.x += param_indent;

    // need to re-draw
    m_context->postRedraw();
}

