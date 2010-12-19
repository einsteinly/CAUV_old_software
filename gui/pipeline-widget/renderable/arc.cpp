#include "arc.h"

#include <QtOpenGL>

#include <debug/cauv_debug.h>

#include "nodeIO.h"

using namespace cauv::pw;

Arc::Arc(container_ptr_t c, renderable_wkptr_t src, renderable_wkptr_t dst)
    : Renderable(c), m_src(src), m_dst(dst), m_hanging(false){
}

// TODO: all the dynamic casting is a bit iffy: if this is only going to work
// with nodeoutput/nodeinputs, then that's what should be stored

void Arc::draw(drawtype_e::e flags){
    if(flags & drawtype_e::picking)
        return;
    if(m_hanging){
        if(!m_src.expired() && !m_dst.expired()){
            debug() << "no longer hanging" << this;
            m_hanging = false;
        }else{
            return;
        }
    }else if(m_src.expired() || m_dst.expired()){
        //warning() << "hanging arc" << this;
        m_hanging = true;
        return;
    }
    renderable_ptr_t src = m_src.lock();
    renderable_ptr_t dst = m_dst.lock();
    
    // TODO: what level should arcs be drawn at?
    glTranslatef(0, 0, 0.05);

    glColor(Colour(1.0, 0.5));
    glLineWidth(1);
    glBegin(GL_LINE_STRIP);
    
    bool src_out = !!boost::dynamic_pointer_cast<NodeOutputBlob>(src);
    bool dst_in =  !!boost::dynamic_pointer_cast<NodeInputBlob>(dst);
    bool src_in =  !!boost::dynamic_pointer_cast<NodeInputBlob>(src);
    bool dst_out = !!boost::dynamic_pointer_cast<NodeOutputBlob>(dst);
    
    Point sp = src->topLevelPos();
    Point dp = dst->topLevelPos();
    Point delta(fabs(sp.x-dp.x)/5 + 20, 0);
    
    if(src_out && dst_in)      glBezier(sp, sp + delta, dp - delta, dp);
    else if(src_in && dst_out) glBezier(sp, sp - delta, dp + delta, dp);
    else if(src_in)            glBezier(sp, sp - delta, dp);
    else if(src_out)           glBezier(sp, sp + delta, dp);
    else if(dst_in)            glBezier(dp, dp - delta, sp);
    else if(dst_out)           glBezier(dp, dp + delta, sp);
    
    glEnd();

    glTranslatef(sp);
    glCircle(4.0);
    glCircleOutline(3.0);

    glTranslatef(dp - sp);
    glCircle(4.0);
    glCircleOutline(3.0);
}

bool Arc::acceptsMouseEvents(){
    return false;
}

cauv::NodeOutput Arc::from(){
    boost::shared_ptr<NodeOutputBlob> optr;
    cauv::NodeOutput r (0, "", OutputType::Image);
    if(optr = boost::dynamic_pointer_cast<NodeOutputBlob>(fromOutput())){
        r.node = optr->nodeId();
        r.output = optr->output();
    }
    return r;
}

cauv::NodeInput Arc::to(){
    boost::shared_ptr<NodeInputBlob> iptr;
    cauv::NodeInput r (0, "");
    if(iptr = boost::dynamic_pointer_cast<NodeInputBlob>(toInput())){
        r.node = iptr->nodeId();
        r.input = iptr->input();
    }
    return r;
}


renderable_ptr_t Arc::fromOutput(){
    renderable_ptr_t sptr = m_src.lock();
    renderable_ptr_t dptr = m_dst.lock();
    boost::shared_ptr<NodeOutputBlob> optr;
    if(!((optr = boost::dynamic_pointer_cast<NodeOutputBlob>(sptr)) ||
         (optr = boost::dynamic_pointer_cast<NodeOutputBlob>(dptr)))){
        warning() << "Arc::fromOutput() neither src nor dst is a NodeOutput";
    }
    return optr;
}

renderable_ptr_t Arc::toInput(){
    renderable_ptr_t sptr = m_src.lock();
    renderable_ptr_t dptr = m_dst.lock();
    boost::shared_ptr<NodeInputBlob> iptr;
    if(!((iptr = boost::dynamic_pointer_cast<NodeInputBlob>(sptr)) ||
         (iptr = boost::dynamic_pointer_cast<NodeInputBlob>(dptr)))){
        warning() << "Arc::fromOutput() neither src nor dst is a NodeInput";
    }
    return iptr;
}
