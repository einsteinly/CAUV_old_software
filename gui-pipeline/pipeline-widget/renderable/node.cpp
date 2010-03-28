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
            glTranslatef(m_param->m_pos_x, m_param->m_pos_y, 0);
            m_param->draw(picking);
            glPopMatrix();
            
            glPushMatrix();
            glTranslatef(m_value->m_pos_x, m_value->m_pos_y, 0);
            m_value->draw(picking);
            glPopMatrix();
        }

        virtual void mousePressEvent(MouseEvent const& e){
            MouseEvent refered(e, m_value);
            if(m_value->bbox().contains(refered.x, refered.y)){
                debug() << "got value hit";
                m_parent.addMenu(
                    boost::make_shared< EditText<tempf> >(
                        boost::ref(m_parent), *m_value, m_value->bbox()
                    ), 0, 0 /* TODO: somehow:
                       refer m_pos_x and m_pos_y to toplevel: probably need
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
            m_value->m_pos_x = m_bbox.xmax + 3 - m_value->bbox().xmin;
            m_bbox.xmax = m_value->m_pos_x + m_value->bbox().xmax;
            if(m_value->bbox().ymin < m_bbox.ymin) m_bbox.ymin = m_value->bbox().ymin;
            if(m_value->bbox().ymax > m_bbox.ymax) m_bbox.ymax = m_value->bbox().ymax;
        }

        BBox m_bbox;

        boost::shared_ptr<Text> m_param;
        boost::shared_ptr<Text> m_value;
};


Node::Node(PipelineWidget& p, boost::shared_ptr<NodeAddedMessage const> m)
    : Draggable(p), m_bbox(), m_node_id(m->nodeId()),
      m_node_type(to_string(m->nodeType())),
      m_title(boost::make_shared<Text>(boost::ref(p), m_node_type)){
    m_bbox.xmin = 0;
    m_bbox.xmax = m_title->bbox().xmax - m_title->bbox().xmin;
    m_bbox.ymax = m_title->bbox().ymax - m_title->bbox().ymin;
    m_bbox.ymin = 0;
    m_title->m_pos_x = -m_title->bbox().xmin;
    m_title->m_pos_y = -m_title->bbox().ymax;
}


void Node::setParams(boost::shared_ptr<NodeParametersMessage const> m){
    if(m_node_id != m->nodeId()){
        warning() << "parameters not for this node";
        return;
    }
    double lead = 2;
    double max_x = m_bbox.xmax;
    double min_x = m_bbox.xmin;
    double prev_height = m_title->bbox().ymax - m_title->bbox().ymin;
    double y_pos = -prev_height-lead;
    std::map<std::string, NodeParamValue>::const_iterator i;    
    for(i = m->values().begin(); i != m->values().end(); i++, y_pos -= (prev_height+lead)){
        boost::shared_ptr<Renderable> t;
        switch((ParamType::e)i->second.type){
            case ParamType::Int32:
                t = boost::make_shared<PVPair<int> >(
                        boost::ref(m_parent), i->first, i->second.intValue
                    );
                break;
            case ParamType::Float:
                t = boost::make_shared<PVPair<float> >(
                        boost::ref(m_parent), i->first, i->second.floatValue
                    );
                break;
            default:
                error() << "unknown ParamType";
            case ParamType::String:
                t = boost::make_shared<PVPair<std::string> >(
                        boost::ref(m_parent), i->first, i->second.stringValue
                    );
                break;
        }
        m_params.push_back(t);
        t->m_pos_y = y_pos + t->bbox().ymin;
        prev_height = t->bbox().ymax - t->bbox().ymin;
        if(t->bbox().xmax > max_x) max_x = t->bbox().xmax;
        if(t->bbox().xmin < min_x) min_x = t->bbox().xmin;
    }
    m_bbox.xmin = min_x;
    m_bbox.xmax = max_x;
    if(m_params.size())
        m_bbox.ymin = m_params.back()->bbox().ymin + y_pos;
    m_bbox.ymax = m_title->bbox().ymax;
    // need to re-draw
    m_parent.updateGL();
}

void Node::mousePressEvent(MouseEvent const& e){
    pv_list_t::const_iterator i;
    for(i = m_params.begin(); i != m_params.end(); i++){
        MouseEvent refered(e, *i);
        if((*i)->bbox().contains(refered.x, refered.y))
            (*i)->mousePressEvent(refered);
    }
    Draggable::mousePressEvent(e);
}

void Node::draw(bool picking){
    if(m_mouseover)
        glColor4f(1.0, 0.0, 0.0, 0.5);
    else
        glColor4f(1.0, 1.0, 1.0, 0.5);
    glBegin(GL_QUADS);
    glVertex2f(m_bbox.xmin, m_bbox.ymax);
    glVertex2f(m_bbox.xmin, m_bbox.ymin);
    glVertex2f(m_bbox.xmax, m_bbox.ymin);
    glVertex2f(m_bbox.xmax, m_bbox.ymax);
    glEnd();

    m_title->draw(picking);
    pv_list_t::const_iterator i;
    for(i = m_params.begin(); i != m_params.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos_x, (*i)->m_pos_y, 0);
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

