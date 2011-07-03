#include "node.h"

#include <sstream>
#include <algorithm>

#include <QtOpenGL>

#include <utility/string.h>
#include <generated/messages.h>
#include <debug/cauv_debug.h>

#include "../pipelineWidget.h"
#include "text.h"
#include "editText.h"
#include "arc.h"
#include "nodeIO.h"
#include "pvPair.h"
#include "buttons.h"

namespace cauv{
namespace pw{

template<typename T, typename cT>
std::basic_ostream<T, cT>& operator<<(
    std::basic_ostream<T, cT>& os, Node const& n){
    return os << "Node{" << n.m_node_type << ",id=" << n.m_node_id << "}";
}

} // namespace pw
} // namespace cauv

using namespace cauv;
using namespace cauv::pw;

const static Colour Mouseover_Colour_Hint(1, 1, 1, 0.2);
const static Colour Normal_BG_Colour(0.7, 0.7, 0.7, 0.8);
const static Colour Queued_Hint(0.3, 0.3, 0, 0.2);
const static Colour Executing_Hint(0.0, 0.4, 0.1, 0.2);
const static Colour Queue_Not_Permitted_Hint(0.4, 0, 0, 0.2);

Node::Node(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m)
    : Draggable(c), m_pw(pw), m_bbox(), m_node_id(m->nodeId()),
      m_node_type(m->nodeType()),
      m_title(boost::make_shared<Text>(c, toStr(m_node_type))),
      m_closebutton(boost::make_shared<CloseButton<Node> >(this)),
      m_execbutton(boost::make_shared<ExecButton>(this)),
      m_suppress_draggable(false),
      m_bg_col(Normal_BG_Colour){
    m_contents.push_back(m_closebutton);
    m_contents.push_back(m_execbutton);
    m_contents.push_back(m_title);

    setOutputs(m->outputs());
    setOutputLinks(m->outputs());

    setParams(m->params());
    setParamLinks(m->inputs()); // param links are inputs

    setInputs(m->inputs());
    setInputLinks(m->inputs());
}

Node::Node(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType::e const& nt)
    : Draggable(c), m_pw(pw), m_bbox(), m_node_id(id),
      m_node_type(nt),
      m_title(boost::make_shared<Text>(c, toStr(m_node_type))),
      m_closebutton(boost::make_shared<CloseButton<Node> >(this)),
      m_execbutton(boost::make_shared<ExecButton>(this)),
      m_suppress_draggable(false),
      m_bg_col(Normal_BG_Colour){
    m_contents.push_back(m_closebutton);
    m_contents.push_back(m_execbutton);
    m_contents.push_back(m_title);
    refreshLayout();
}

void Node::setType(NodeType::e const& n){
    m_node_type = n;
    m_title = boost::make_shared<Text>(m_context, toStr(m_node_type));
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
        // don't add any inputs that are actually parameters!
        // TODO: fix the order-dependence here... really parameters should have
        // been filtered out by the time this function is called
        if(!m_params.count(j->first)){
            debug() << BashColour::Blue << "Node::" << __func__ << *j;
            in_ptr_t t = boost::make_shared<NodeInputBlob>(
                this, m_pw, j->first
            );
            m_inputs[j->first] = t;
            m_contents.push_back(t);
        }
    }
    // NB: layout not refreshed
}

void Node::setInputLinks(std::map<std::string, NodeOutput> const& inputs){
    std::map<std::string, NodeOutput>::const_iterator j;
    for(j = inputs.begin(); j != inputs.end(); j++){
        str_in_map_t::const_iterator k = m_inputs.find(j->first);
        // if connected, add the arc to this input:
        if(k != m_inputs.end() && j->second.node)
            m_pw->addArc(j->second.node, j->second.output, k->second);
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
        if(m_outputs.count(i->first)){
            renderable_ptr_t t = m_outputs[i->first];
            std::vector<NodeInput>::const_iterator k;
            for(k = i->second.begin(); k != i->second.end(); k++){
                m_pw->addArc(t, k->node, k->input);
            }
        }else{
            error() << "output link specified for non-existent output:"
                    << m_node_type << m_node_id << i->first;
        }
    }
    refreshLayout();
}

namespace{//Unnamed

template<typename T> struct Type2Name { static const char* name; };
template<typename T> const char* Type2Name<T>::name = "Unknown";
template<> const char* Type2Name<cauv::KeyPoint>::name = "KeyPoint";
template<> const char* Type2Name<Corner>::name = "Corner";
template<> const char* Type2Name<Line>::name = "Line";
template<> const char* Type2Name<float>::name = "float";

template<typename T>
struct makePVPairHelper
{
};
template<typename T>
struct makePVPairHelper<const T>
{
    static boost::shared_ptr<PVPairEditableBase> exec(Node *n, const std::string& name, const T& val, bool editable) {
        return boost::make_shared< PVPair<T> >(n, name, val, editable);
    }
};
template<typename T>
struct makePVPairHelper< const std::vector<T> >
{
    static boost::shared_ptr<PVPairEditableBase> exec(Node *n, const std::string& name, const std::vector<T>& val, bool) {
        std::stringstream ss;
        ss << Type2Name<T>::name << "[" << val.size() << "]"; 
        return boost::make_shared< PVPair<std::string> >(n, name, ss.str(), false);
    }
};

struct PVPairVisitor: public boost::static_visitor< boost::shared_ptr<PVPairEditableBase> >
{
public:
    PVPairVisitor(Node* n, const std::string& name, bool editable) : m_n(n), m_name(name), m_editable(editable)
    {
    }

    template<typename T>
    boost::shared_ptr<PVPairEditableBase> operator()(T & val) const
    {
        return makePVPairHelper<T>::exec(m_n, m_name, val, m_editable);
    }
    
protected:
    Node* m_n;
    const std::string& m_name;
    bool m_editable;

};

boost::shared_ptr<PVPairEditableBase> makePVPair(
    Node *n, std::pair<std::string, NodeParamValue> const& p, bool editable){
    return boost::apply_visitor(PVPairVisitor(n, p.first, editable), p.second);
}

}// namespace Unnamed

void Node::setParams(std::map<std::string, NodeParamValue> const& params){
    // remove any parameters that no longer exist
    str_inparam_map_t new_m_params;
    foreach(str_inparam_map_t::value_type const &i, m_params)
        if(!params.count(i.first)){
            m_contents.remove(i.second.inblob);
            m_contents.remove(i.second.pvpair);
            m_pw->removeArc(i.second.inblob, m_node_id, i.first);
        }else{
            new_m_params.insert(i);
        }
    m_params = new_m_params;

    typedef std::map<std::string, NodeParamValue> pm_t; // TODO auto
    foreach(pm_t::value_type const& j, params){
        str_inparam_map_t::iterator k = m_params.find(j.first);
        if(k == m_params.end()){
            debug(3) << BashColour::Blue << *this << "new param:" << j;
            InParamPVPair t;
            t.pvpair = makePVPair(this, j, true);
            t.inblob = boost::make_shared<NodeInputParamBlob>(
                this, m_pw, j.first
            );
            m_params[j.first] = t;
            m_contents.push_back(t.inblob);
            m_contents.push_back(t.pvpair);
        }else{
            debug(3) << BashColour::Blue << *this << "param updated:" << j;        
            // leave the 'inblob' alone -- so that any input arc to it remains
            // valid -- but replace the parameter-value-pair:
            renderable_list_t::iterator i = std::find(
                m_contents.begin(), m_contents.end(), k->second.pvpair
            );
            assert(i != m_contents.end());
            assert(k->second.pvpair);
            k->second.pvpair = makePVPair(this, j, k->second.pvpair->editable());
            *i = k->second.pvpair;
        }
    }
    refreshLayout();
}

void Node::setParamLinks(std::map<std::string, NodeOutput> const& inputs){
    typedef std::map<std::string, NodeOutput> im_t;
    foreach(im_t::value_type const& j, inputs){
        str_inparam_map_t::const_iterator k = m_params.find(j.first);
        // if connected, add the arc to this parameter's input:
        if(k != m_params.end() && j.second.node){
            m_pw->addArc(j.second.node, j.second.output, k->second.inblob);
            k->second.pvpair->editable(false);
        }else if(k != m_params.end()){
            k->second.pvpair->editable(true);
        }
    }
    refreshLayout();
}

void Node::setParams(boost::shared_ptr<NodeParametersMessage const> m){
    if(m_node_id != m->nodeId()){
        error() << "parameters not for this node";
        return;
    }
    setParams(m->params());
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
        }else if(m_pressed.count(*i)){
            (*i)->mouseMoveEvent(referred);
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

void Node::draw(drawtype_e::e flags){
    if(m_mouseover)
        glColor(m_bg_col & Mouseover_Colour_Hint);
    else
        glColor(m_bg_col);
    glBox(m_back, 6);

    Container::draw(flags);
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

NodeType::e Node::type() const{
    return m_node_type;
}

void Node::close(){
    debug() << "Node::close" << id();
    m_pw->send(boost::make_shared<RemoveNodeMessage>(m_pw->pipelineName(), m_node_id));
}

void Node::exec(){
    debug() << "Node::exec" << id();
    m_pw->send(boost::make_shared<ForceExecRequestMessage>(m_pw->pipelineName(), m_node_id));
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
        // could be a parameter input:
        str_inparam_map_t::const_iterator j = m_params.find(id);
        if(j != m_params.end()){
            return j->second.inblob;
        }else{
            warning() << "unknown input:" << id << "on node" << m_node_id;
            return renderable_ptr_t();
        }
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
    m_context->postRedraw(0);
}

void Node::inputStatus(std::string const& input_id, int s){
    str_in_map_t::const_iterator i = m_inputs.find(input_id);
    if(i == m_inputs.end()){
        str_inparam_map_t::const_iterator j = m_params.find(input_id);
        if(j != m_params.end()){
            j->second.inblob->status(s);
        }else{
            warning() << input_id << "is not input or parameter id on this node";
        }
    }else{
        if(i->second)
            i->second->status(s);
    }
}

void Node::outputStatus(std::string const& output_id, int s){
    str_out_map_t::const_iterator i = m_outputs.find(output_id);
    assert(i != m_outputs.end());
    if(i->second)
        i->second->status(s);
}

Point Node::referUp(Point const& p) const{
    return m_context->referUp(p + m_pos);
}

void Node::postRedraw(float delay){
    m_context->postRedraw(delay);
}

void Node::postMenu(menu_ptr_t m, Point const& p, bool r){
    m_context->postMenu(m, p, r);
}

void Node::postText(const std::string &text, const std::string &font)
{
	m_context->postText(text,font);
}

void Node::removeMenu(menu_ptr_t m){
    m_context->removeMenu(m);
}

void Node::remove(renderable_ptr_t){
    error() << __func__ << __LINE__ << "unimplemented";
}

void Node::refreshLayout(){
    // yay, lots of random constants: layout is fun
    const int border = 3;
    const int lead = 6;
    const int section_lead = 10;
    const int io_overhang = 6;
    const int param_io_overhang = 4;
    const int param_indent = 6;
    int prev_height = roundA(m_closebutton->bbox().h());
    int y_pos = -prev_height;

    m_closebutton->m_pos.x = -m_closebutton->bbox().min.x;
    m_closebutton->m_pos.y = -m_closebutton->bbox().min.y;
    m_back = m_closebutton->bbox() + m_closebutton->m_pos;
    debug(2) << "closebutton layout:" << m_closebutton->m_pos << m_closebutton->bbox();

    m_title->m_pos.y = y_pos - roundA(m_title->bbox().max.y);
    m_title->m_pos.x = -m_title->bbox().min.x;
    prev_height = roundA(m_title->bbox().h());
    y_pos -= (prev_height + lead);

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
    if(m_inputs.size())
        y_pos -= section_lead - lead;

    str_inparam_map_t::const_iterator j;
    for(j = m_params.begin(); j != m_params.end(); j++, y_pos -= (prev_height+lead)){
        renderable_ptr_t blob = j->second.inblob;
        renderable_ptr_t pvp = j->second.pvpair;
        
        blob->m_pos.y = y_pos - roundA(std::max(pvp->bbox().max.y, blob->bbox().max.y));
        pvp->m_pos.y = blob->m_pos.y;

        blob->m_pos.x = m_back.min.x - param_io_overhang - blob->bbox().min.x;
        pvp->m_pos.x = param_indent - pvp->bbox().min.x;

        prev_height = roundA(std::max(pvp->bbox().max.y, blob->bbox().max.y) -
                             std::min(pvp->bbox().min.y, blob->bbox().min.y));

        m_back |= blob->bbox() + blob->m_pos + Point(io_overhang, 0);
        m_back |= pvp->bbox() + pvp->m_pos;
    }
    if(m_params.size())
        y_pos -= section_lead - lead;


    for(renderable_set_t::const_iterator j = m_extra_stuff.begin();
        j != m_extra_stuff.end(); j++, y_pos -= (prev_height+lead))
    {
        renderable_ptr_t r = *j;
        r->m_pos.y = y_pos - roundA(r->bbox().max.y);
        r->m_pos.x = param_indent;
        prev_height = roundA(r->bbox().h());
        m_back |= r->bbox() + r->m_pos;
    }
    if(m_extra_stuff.size())
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
        prev_height = roundA(r->bbox().h());    }

    m_back.min.y -= border;

    m_bbox = m_back;
    if(m_inputs.size())
        m_bbox.min.x -= param_indent;
    if(m_outputs.size())
        m_bbox.max.x += param_indent;

    m_execbutton->m_pos.x = m_back.max.x - m_execbutton->bbox().max.x - border;
    m_execbutton->m_pos.y = m_back.max.y - m_execbutton->bbox().max.y - border;

    debug(2) << "execbutton layout:" << m_execbutton->m_pos << m_execbutton->bbox();


    // need to re-draw
    m_context->postRedraw(0);
}

