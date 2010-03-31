#include "arc.h"

#include <QtOpenGL>

#include <common/messages.h>
#include <common/debug.h>

#include "NodeIO.h"

using namespace pw;

Arc::Arc(container_ptr_t c, renderable_wkptr_t src, renderable_wkptr_t dst)
    : Renderable(c), m_src(src), m_dst(dst), m_hanging(false){
}

void Arc::draw(bool picking){
    if(picking)
        return;
    if(m_hanging){
        if(!m_src.expired() && !m_dst.expired()){
            debug() << "no longer hanging" << this;
            m_hanging = false;
        }else{
            return;
        }
    }else if(m_src.expired() || m_dst.expired()){
        warning() << "hanging arc" << this;
        m_hanging = true;
        return;
    }
    renderable_ptr_t src = m_src.lock();
    renderable_ptr_t dst = m_dst.lock();
    
    // TODO: what level should arcs be drawn at?
    glTranslatef(0, 0, 0.05);

    glColor(Colour(1.0, 0.5));
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex(src->topLevelPos());
    glVertex(dst->topLevelPos());
    glEnd();

    glTranslatef(src->topLevelPos());
    glCircle(4.0);
    glCircleOutline(3.0);

    glTranslatef(dst->topLevelPos() - src->topLevelPos());
    glCircle(4.0);
    glCircleOutline(3.0);
}

bool Arc::acceptsMouseEvents(){
    return false;
}

NodeOutput Arc::from(){
    renderable_ptr_t sptr = m_src.lock();
    renderable_ptr_t dptr = m_dst.lock();
    boost::shared_ptr<NodeOutputBlob> iptr;
    NodeOutput r = {0};
    if((iptr = boost::dynamic_pointer_cast<NodeOutputBlob>(sptr)) ||
       (iptr = boost::dynamic_pointer_cast<NodeOutputBlob>(dptr))){
        r.node = iptr->nodeId();
        r.output = iptr->output();
    }else{
        warning() << "Arc::from() neither src nor dst is a NodeOutput";
    }
    return r;
}

NodeInput Arc::to(){
    renderable_ptr_t sptr = m_src.lock();
    renderable_ptr_t dptr = m_dst.lock();
    boost::shared_ptr<NodeInputBlob> iptr;
    NodeInput r = {0};
    if((iptr = boost::dynamic_pointer_cast<NodeInputBlob>(sptr)) ||
       (iptr = boost::dynamic_pointer_cast<NodeInputBlob>(dptr))){
        r.node = iptr->nodeId();
        r.input = iptr->input();
    }else{
        warning() << "Arc::to() neither src nor dst is a NodeInput";
    }
    return r;
}

