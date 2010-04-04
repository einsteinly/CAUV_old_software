#include "node.h"

#include <sstream>
#include <algorithm>

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
            m_sort_key = param;
        }
        virtual ~PVPair(){ }

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

template<typename container_T>
class CloseButton: public Renderable{
    public:
        CloseButton(container_T* c)
            : Renderable(c), m_container(c), m_size(8), m_mouseover(false),
              m_pressed(false){
        }
        virtual ~CloseButton(){ }

        virtual void draw(bool){
            if(m_pressed)
                glColor(Colour(0, 0.9));
            else if(m_mouseover)
                glColor(Colour(0.8, 0, 0, 0.8));
            else
                glColor(Colour(0, 0.5));
            glLineWidth(roundA(m_size/4));
            glBegin(GL_LINES);
            glVertex2f(-m_size/2, m_size/2);
            glVertex2f(m_size/2, -m_size/2);
            glVertex2f(-m_size/2, -m_size/2);
            glVertex2f(m_size/2, m_size/2);
            glEnd();
        }

        virtual void mouseMoveEvent(MouseEvent const& event){
            if(bbox().contains(event.pos) && !m_mouseover){
                m_mouseover = true;
                m_context->postRedraw();
            }
        }

        virtual bool mousePressEvent(MouseEvent const& event){
            if(bbox().contains(event.pos)){
                m_pressed = true;
                m_context->postRedraw();
                return true;
            }
            return false;
        }

        virtual void mouseReleaseEvent(MouseEvent const& event){
            if(m_pressed && bbox().contains(event.pos)){
                m_container->close();
            }
            m_pressed = false;
            m_context->postRedraw();
        }

        virtual void mouseGoneEvent(){
            if(m_pressed || m_mouseover){
                m_pressed = false;
                m_mouseover = false;
                m_context->postRedraw();
            }
        }

        virtual bool tracksMouse(){
            return true;
        }

        virtual BBox bbox(){
            return BBox(-m_size/2, -m_size/2, m_size/2, m_size/2);
        }

     private:
        container_T* m_container;
        double m_size;
        bool m_mouseover;
        bool m_pressed;
};

} // namespace pw

using namespace pw;

const static Colour Mouseover_Colour_Hint(1, 1, 1, 0.2);
const static Colour Normal_BG_Colour(0.7, 0.7, 0.7, 0.8);
const static Colour Queued_Hint(0.3, 0.3, 0, 0.2);
const static Colour Executing_Hint(0.0, 0.4, 0.1, 0.2);
const static Colour Queue_Not_Permitted_Hint(0.4, 0, 0, 0.2);

Node::Node(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m)
    : Draggable(c), m_pw(pw), m_bbox(), m_node_id(m->nodeId()),
      m_node_type(to_string(m->nodeType())),
      m_title(boost::make_shared<Text>(c, m_node_type)),
      m_closebutton(boost::make_shared<CloseButton<Node> >(this)),
      m_suppress_draggable(false),
      m_bg_col(Normal_BG_Colour){
    m_contents.push_back(m_closebutton);
    m_contents.push_back(m_title);

    setOutputs(m->outputs());
    setOutputLinks(m->outputs());

    setInputs(m->inputs());
    setInputLinks(m->inputs());
}

Node::Node(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType::e const& nt)
    : Draggable(c), m_pw(pw), m_bbox(), m_node_id(id),
      m_node_type(to_string(nt)),
      m_title(boost::make_shared<Text>(c, m_node_type)),
      m_closebutton(boost::make_shared<CloseButton<Node> >(this)),
      m_suppress_draggable(false),
      m_bg_col(Normal_BG_Colour){
    m_contents.push_back(m_closebutton);
    m_contents.push_back(m_title);
    refreshLayout();
}

void Node::setType(NodeType::e const& n){
    m_node_type = to_string(n);
    m_title = boost::make_shared<Text>(m_context, m_node_type);
    refreshLayout();
}

void Node::setInputs(std::map<std::string, NodeOutput> const& inputs){
    // remove any old inputs:
    for(str_in_map_t::const_iterator i = m_inputs.begin(); i != m_inputs.end(); i++){
        m_contents.remove(i->second);
        m_pw->removeArc(i->second, m_node_id, i->first);
    }
    m_inputs.clear();

    std::map<std::string, NodeOutput>::const_iterator j;
    for(j = inputs.begin(); j != inputs.end(); j++){
        debug() << BashColour::Blue << "Node::" << __func__ << *j;
        in_ptr_t t = boost::make_shared<NodeInputBlob>(
            this, m_pw, j->first
        );
        m_inputs[j->first] = t;
        m_contents.push_back(t);
    }
    // NB: layout not refreshed
}

void Node::setInputLinks(std::map<std::string, NodeOutput> const& inputs){
    std::map<std::string, NodeOutput>::const_iterator j;
    for(j = inputs.begin(); j != inputs.end(); j++){
        // if connected, add the arc to this input:
        renderable_ptr_t t = m_inputs[j->first];
        if(j->second.node)
            m_pw->addArc(j->second.node, j->second.output, t);
    }
    refreshLayout();
}

void Node::setOutputs(std::map<std::string, std::vector<NodeInput> > const& outputs){
    // remove any old outputs:
    for(str_out_map_t::const_iterator i = m_outputs.begin(); i != m_outputs.end(); i++){
        m_contents.remove(i->second);
        m_pw->removeArc(m_node_id, i->first, i->second);
    }
    m_outputs.clear();

    std::map<std::string, std::vector<NodeInput> >::const_iterator i;
    for(i = outputs.begin(); i != outputs.end(); i++){
        debug() << BashColour::Blue << "Node::" << __func__ << *i;
        out_ptr_t t = boost::make_shared<NodeOutputBlob>(
            this, m_pw, i->first
        );
        m_outputs[i->first] = t;
        m_contents.push_back(t);
    }
    // NB: layout not refreshed
}

void Node::setOutputLinks(std::map<std::string, std::vector<NodeInput> > const& outputs){
    std::map<std::string, std::vector<NodeInput> >::const_iterator i;
    for(i = outputs.begin(); i != outputs.end(); i++){
        // add the arcs (if any) from this output:
        renderable_ptr_t t = m_outputs[i->first];
        std::vector<NodeInput>::const_iterator k;
        for(k = i->second.begin(); k != i->second.end(); k++){
            m_pw->addArc(t, k->node, k->input);
        }
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
        case ParamType::String:
            return boost::make_shared<PVPair<std::string> >(
                    n, p.first, p.second.stringValue
                );
        default:
            error() << "unknown ParamType";
        case ParamType::Bool:
            return boost::make_shared<PVPair<bool> >(
                    n, p.first, p.second.boolValue
                );
    }
}

void Node::setParams(std::map<std::string, NodeParamValue> const& params){
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
    for(i = params.begin(); i != params.end(); i++){
        debug() << BashColour::Blue << "Node::" << __func__ << *i;
        boost::shared_ptr<Renderable> t = makePVPair(this, *i);
        m_params.insert(t);
        m_contents.push_back(t);
    }
    refreshLayout();
}

void Node::setParams(boost::shared_ptr<NodeParametersMessage const> m){
    if(m_node_id != m->nodeId()){
        error() << "parameters not for this node";
        return;
    }
    setParams(m->values());
}

bool Node::mousePressEvent(MouseEvent const& e){
    renderable_list_t::const_iterator i;
    m_pressed.clear();
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        MouseEvent referred(e, *i);
        if((*i)->bbox().contains(referred.pos)){
            m_pressed.insert(*i);
            if((*i)->mousePressEvent(referred)){
                m_suppress_draggable = true;
                return true;
            }
        }
    }
    return Draggable::mousePressEvent(e);
}

void Node::mouseReleaseEvent(MouseEvent const& e){
    m_suppress_draggable = false;
    renderable_set_t::const_iterator j;
    for(j = m_pressed.begin(); j != m_pressed.end(); j++){
        MouseEvent referred(e, *j);
        (*j)->mouseReleaseEvent(referred);
    }
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
        glColor(m_bg_col & Mouseover_Colour_Hint);
    else
        glColor(m_bg_col);
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

void Node::close(){
    debug() << "Node::close" << id();
    m_pw->sendMessage(boost::make_shared<RemoveNodeMessage>(m_node_id));
}

renderable_ptr_t Node::outSocket(std::string const& id){
    str_out_map_t::const_iterator i = m_outputs.find(id);
    if(i != m_outputs.end()){
        return i->second;
    }else{
        warning() << "unknown output:" << id << "on node" << m_node_id;
        return renderable_ptr_t();
    }
}

renderable_ptr_t Node::inSocket(std::string const& id){
    str_in_map_t::const_iterator i = m_inputs.find(id);
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

void Node::status(int s){
    m_bg_col = Normal_BG_Colour;
    if(s & NodeStatus::Executing)
        m_bg_col &= Executing_Hint;
    if(!(s & NodeStatus::AllowQueue))
        m_bg_col &= Queue_Not_Permitted_Hint;
    if(s & NodeStatus::ExecQueued)
        m_bg_col &= Queued_Hint;
    m_context->postRedraw();
}

void Node::inputStatus(std::string const& input_id, int s){
    str_in_map_t::const_iterator i = m_inputs.find(input_id);
    if(i->second)
        i->second->status(s);
}

void Node::outputStatus(std::string const& output_id, int s){
    str_out_map_t::const_iterator i = m_outputs.find(output_id);
    if(i->second)
        i->second->status(s);
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

void Node::remove(renderable_ptr_t){
    error() << __func__ << __LINE__ << "unimplemented";
}

namespace pw{

template<>
void Node::paramValueChanged<int>(std::string const& p, int const& v){
    debug() << "Node::paramValueChanged<int>" << p << v;
    boost::shared_ptr<SetNodeParameterMessage> sp =
        boost::make_shared<SetNodeParameterMessage>();
    NodeParamValue pv = {0,0,0,"",0};

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
    NodeParamValue pv = {0,0,0,"",0};

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
    NodeParamValue pv = {0,0,0,"",0};

    sp->nodeId(m_node_id);
    sp->paramId(p);
    pv.type = ParamType::String;
    pv.stringValue = v;
    sp->value(pv);
    m_pw->sendMessage(sp);
}

template<>
void Node::paramValueChanged<bool>(std::string const& p, bool const& v){
    debug() << "Node::paramValueChanged<string>" << p << v;
    boost::shared_ptr<SetNodeParameterMessage> sp =
        boost::make_shared<SetNodeParameterMessage>();
    NodeParamValue pv = {0,0,0,"",0};

    sp->nodeId(m_node_id);
    sp->paramId(p);
    pv.type = ParamType::Bool;
    pv.boolValue = v;
    sp->value(pv);
    m_pw->sendMessage(sp);
}

} // namespace pw

void Node::refreshLayout(){
    int border = 3;
    int lead = 6;
    int section_lead = 10;
    int io_overhang = 6;
    int prev_height = roundA(m_closebutton->bbox().h());
    int y_pos = -prev_height;
    int param_indent = 6;

    m_closebutton->m_pos.x = -m_closebutton->bbox().min.x;
    m_closebutton->m_pos.y = -m_closebutton->bbox().min.y;
    m_back = m_closebutton->bbox() + m_closebutton->m_pos;
    debug(-1) << "closebutton layout:" << m_closebutton->m_pos << m_closebutton->bbox();

    m_title->m_pos.y = y_pos - roundA(m_title->bbox().max.y);
    m_title->m_pos.x = -m_title->bbox().min.x;
    prev_height = roundA(m_title->bbox().h());
    y_pos -= (prev_height + section_lead);

    m_back |= m_title->bbox() + m_title->m_pos;
    m_back.max.y += border;
    m_back.min.x -= border;

    str_in_map_t::const_iterator i;
    for(i = m_inputs.begin(); i != m_inputs.end(); i++, y_pos -= (prev_height+lead)){
        renderable_ptr_t r = i->second;
        r->m_pos.y = y_pos - roundA(r->bbox().max.y);
        r->m_pos.x = m_back.min.x - io_overhang - r->bbox().min.x;
        prev_height = roundA(r->bbox().h());
        m_back |= r->bbox() + r->m_pos + Point(io_overhang, 0);
    }
    y_pos -= section_lead - lead;

    // stop parameter order from jumping around:
    // TODO: editing here, this is actually rather a tricky structural
    // problem...
    std::list<renderable_ptr_t> params;
    for(pv_set_t::const_iterator j = m_params.begin(); j != m_params.end(); j++)
        params.push_back(*j);
    params.sort(lessDereferenced<renderable_ptr_t, renderable_ptr_t>);
    std::list<renderable_ptr_t>::const_iterator l;
    for(l = params.begin(); l != params.end(); l++, y_pos -= (prev_height+lead)){
        (*l)->m_pos.y = y_pos - roundA((*l)->bbox().max.y);
        (*l)->m_pos.x = param_indent;
        prev_height = roundA((*l)->bbox().h());
        m_back |= (*l)->bbox() + (*l)->m_pos;
    }
    y_pos -= section_lead - lead;

    m_back.max.x += border;

    str_out_map_t::const_iterator k;
    for(k = m_outputs.begin(); k != m_outputs.end(); k++, y_pos -= (prev_height+lead)){
        renderable_ptr_t r = k->second;
        r->m_pos.y = y_pos - roundA(r->bbox().max.y);
        if(r->bbox().w() - io_overhang > m_back.w())
            r->m_pos.x = m_back.min.x - r->bbox().min.x;
        else
            r->m_pos.x = m_back.max.x + io_overhang - r->bbox().max.x;
        m_back |= r->bbox() + r->m_pos - Point(io_overhang, 0);
        prev_height = roundA(r->bbox().h());
    }

    m_back.min.y -= border;

    m_bbox = m_back;
    if(m_inputs.size())
        m_bbox.min.x -= param_indent;
    if(m_outputs.size())
        m_bbox.max.x += param_indent;

    // need to re-draw
    m_context->postRedraw();
}

