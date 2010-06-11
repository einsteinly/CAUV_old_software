#include "container.h"

#include <QtOpenGL>

#include <common/debug.h>

#include "renderable.h"

using namespace pw;

renderable_ptr_t Container::pick(Point const& p){
    renderable_list_t::const_iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++)
        if((*i)->bbox().contains(p - (*i)->m_pos)){
            boost::shared_ptr<Container> cp;
            // eww, a bit nasty
            if(cp = boost::dynamic_pointer_cast<Container>(*i)){
                debug() << __func__ << "hit container";
                return cp->pick(p - (*i)->m_pos);
            }else{
                debug() << __func__ << "not a container";
                return *i;
            }
        }
    return renderable_ptr_t();
}

void Container::draw(bool picking){
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos);
        (*i)->draw(picking);
        glPopMatrix();
    }
}

BBox Container::bbox(){
    BBox r;
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++)
        r |= (*i)->bbox();
    return r;
}
