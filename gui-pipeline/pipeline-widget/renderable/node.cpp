#include "node.h"
#include "text.h"
#include "../pipelineWidget.h"
#include "editText.h"

#include <common/cauv_utils.h>
#include <common/messages.h>

#include <QtOpenGL>


void tempf(std::string s){
    debug() << "PVPair: Edit done:" << s;
}

// TODO: own header
template<typename value_T>
class PVPair: public Renderable{
    public:
        PVPair(PipelineWidget& p, std::string const& param, value_T const& value)
            : Renderable(p), m_bbox(),
              m_param(boost::make_shared<Text>(boost::ref(p), param + " =")),
              m_value(boost::make_shared<Text>(boost::ref(p), to_string(value))){
            updateBbox();
        }

        void draw(bool picking){
            glPushMatrix();
            glTranslatef(m_param->m_pos);
            m_param->draw(picking);
            glPopMatrix();
            
            glPushMatrix();
            glTranslatef(m_value->m_pos);
            m_value->draw(picking);
            glPopMatrix();
        }

        virtual void mousePressEvent(MouseEvent const& e){
            MouseEvent refered(e, m_value);
            if(m_value->bbox().contains(refered.pos)){
                debug() << "got value hit";
                m_parent.addMenu(
                    boost::make_shared< EditText<tempf> >(
                        boost::ref(m_parent), *m_value, m_value->bbox()
                    ), Point() /* TODO: somehow:
                       refer m_pos to toplevel: probably need
                       m_parent to be base type RenderableContainer (ie, Node
                       derives RenderableContainer and Renderable, and becomes
                       the parent of this), then a chain of m_parents can be
                       followed to refer coordinates back up to the top */
                );
                m_parent.updateGL();
            }
        }

        virtual BBox bbox(){
            return m_bbox;
        }


    private:
        void updateBbox(){
            m_bbox = m_param->bbox();
            m_value->m_pos.x = m_bbox.max.x + 3 - m_value->bbox().min.x;
            m_bbox |= m_value->bbox() + m_value->m_pos;
        }

        BBox m_bbox;

        boost::shared_ptr<Text> m_param;
        boost::shared_ptr<Text> m_value;
};


Node::Node(PipelineWidget& p, boost::shared_ptr<NodeAddedMessage const> m)
    : Draggable(p), m_bbox(), m_node_id(m->nodeId()),
      m_node_type(to_string(m->nodeType())),
      m_title(boost::make_shared<Text>(boost::ref(p), m_node_type)){
    m_title->m_pos.x = -m_title->bbox().min.x;
    m_title->m_pos.y = -m_title->bbox().max.y;
    m_bbox = m_title->bbox() + m_title->m_pos;
}

static boost::shared_ptr<Renderable> makePVPair(
    PipelineWidget& parent, std::pair<std::string, NodeParamValue> const& p){
    switch((ParamType::e) p.second.type){
        case ParamType::Int32:
            return boost::make_shared<PVPair<int> >(
                    boost::ref(parent), p.first, p.second.intValue
                );
        case ParamType::Float:
            return boost::make_shared<PVPair<float> >(
                    boost::ref(parent), p.first, p.second.floatValue
                );
        default:
            error() << "unknown ParamType";
        case ParamType::String:
            return boost::make_shared<PVPair<std::string> >(
                    boost::ref(parent), p.first, p.second.stringValue
                );
    }
}

void Node::setParams(boost::shared_ptr<NodeParametersMessage const> m){
    if(m_node_id != m->nodeId()){
        warning() << "parameters not for this node";
        return;
    }
    m_bbox = m_title->bbox() + m_title->m_pos;
    double lead = 4;
    double prev_height = m_title->bbox().h();
    double y_pos = -(prev_height+lead);
    std::map<std::string, NodeParamValue>::const_iterator i;    
    for(i = m->values().begin(); i != m->values().end(); i++, y_pos -= (prev_height+lead)){
        boost::shared_ptr<Renderable> t = makePVPair(m_parent, *i);
        m_params.push_back(t);
        t->m_pos.y = y_pos - t->bbox().max.y;
        prev_height = t->bbox().h();
        m_bbox |= t->bbox() + t->m_pos;
    }
    // need to re-draw
    m_parent.updateGL();
}

void Node::mousePressEvent(MouseEvent const& e){
    pv_list_t::const_iterator i;
    for(i = m_params.begin(); i != m_params.end(); i++){
        MouseEvent refered(e, *i);
        if((*i)->bbox().contains(refered.pos))
            (*i)->mousePressEvent(refered);
    }
    Draggable::mousePressEvent(e);
}

void Node::draw(bool picking){
    if(m_mouseover)
        glColor4f(1.0, 0.0, 0.0, 0.5);
    else
        glColor4f(1.0, 1.0, 1.0, 0.5);
    glBox(m_bbox);

    glPushMatrix();
    glTranslatef(m_title->m_pos);
    m_title->draw(picking);
    glPopMatrix();

    pv_list_t::const_iterator i;
    for(i = m_params.begin(); i != m_params.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos);
        (*i)->draw(picking);
        glPopMatrix();
    }
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

